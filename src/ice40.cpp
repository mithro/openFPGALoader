// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#include "ice40.hpp"

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "display.hpp"
#include "ftdispi.hpp"
#include "device.hpp"
#include "progressBar.hpp"
#include "rawParser.hpp"
#include "spiFlash.hpp"

Ice40::Ice40(FtdiSpi* spi, const std::string &filename,
			const std::string &file_type,
			Device::prog_type_t prg_type,
			uint16_t rst_pin, uint16_t done_pin,
			bool verify, int8_t verbose):
	Device(NULL, filename, file_type, verify, verbose), _slave(NULL),
		_rst_pin(rst_pin), _done_pin(done_pin)
{
	_spi = spi;
	_spi->gpio_set_input(_done_pin);
	_spi->gpio_set_output(_rst_pin);

	if (prg_type == Device::WR_FLASH)
		_mode = Device::SPI_MODE;
	else
		_mode = Device::MEM_MODE;
}

Ice40::Ice40(Ice40SpiSlaveInterface *slave, const std::string &filename,
			const std::string &file_type, int8_t verbose):
	Device(NULL, filename, file_type, false, verbose), _spi(NULL),
		_slave(slave), _rst_pin(0), _done_pin(0)
{
	_mode = Device::MEM_MODE;
}

Ice40::~Ice40()
{}

bool Ice40::flash_access_unsupported()
{
	printError("Error: this cable loads the iCE40's SRAM only: "
		"no flash access");
	return false;
}

void Ice40::reset()
{
	if (!_spi)
		throw std::runtime_error("reset: not supported on this cable "
			"(an SRAM load resets the iCE40 itself)");
	uint32_t timeout = 1000;
	_spi->gpio_clear(_rst_pin);
	usleep(1000);
	_spi->gpio_set(_rst_pin);
	usleep(12000);
	printInfo("Reset and Wait for CDONE:");
	do {
		timeout--;
		usleep(12000);
	} while (((_spi->gpio_get(true) & _done_pin) == 0) && timeout > 0);
	if (timeout == 0)
		printError("Fail");
	else
		printSuccess("Done");
}

/* cf. TN1248 (iCE40 Programming and Configuration)
 * Appendix A. SPI Slave Configuration Procedure
 */
bool Ice40::program_cram(const uint8_t *data, uint32_t length)
{
	uint32_t timeout = 1000;

	/* configure SPI */
	_spi->setMode(3); // IDLE high, write on falling
	_spi->setCSmode(FtdiSpi::SPI_CS_MANUAL);

	/* reset device */
	_spi->clearCs();
	_spi->gpio_clear(_rst_pin);
	usleep(100); // 200 ns ...
	_spi->gpio_set(_rst_pin);
	usleep(2000); // 800 -> 1200 us + guard

	/* load configuration data MSB first
	 */
	ProgressBar progress("Loading to CRAM", length, 50, _quiet);
	const uint8_t *ptr = data;
	int size = 0;
	for (uint32_t addr = 0; addr < length; addr += size, ptr+=size) {
		size = (addr + 256 > length)?(length-addr) : 256;
		if (_spi->spi_put(ptr, NULL, size) == -1)
			return -1;
		progress.display(addr);
	}
	progress.done();

	/* send 48 to 100 dummy bits */
	uint8_t dummy[12];
	memset(dummy, 0xff, sizeof(dummy));
	_spi->spi_put(dummy, NULL, 12);

	/* wait CDONE */
	usleep(12000);

	printInfo("Wait for CDONE");
	do {
		timeout--;
		usleep(12000);
	} while (((_spi->gpio_get(true) & _done_pin) == 0) && timeout > 0);
	if (timeout == 0)
		printError("Fail");
	else
		printSuccess("Done");

	_spi->setCs();

	return (timeout == 0) ? false : true;
}

/* iCE40 UltraPlus Family Data Sheet (FPGA-DS-02008 1.5), Table 4.24
 * "sysCONFIG Port Timing Specifications": tCRESET_B min 200 ns; Slave SPI
 * tCR_SCK min 1200 us; Slave SPI fMAX "CCLK clock frequency" Write min
 * 1 MHz, max 25 MHz; tDONE_IO min 49 clock cycles.
 */
#define ICE40_CRESET_LOW_US		1000	/* tCRESET_B >= 200 ns */
#define ICE40_CR_SCK_US			2000	/* tCR_SCK >= 1200 us, + guard */
#define ICE40_SLAVE_SCK_MIN_HZ	1000000.0
#define ICE40_SLAVE_SCK_MAX_HZ	25000000.0
#define ICE40_DONE_IO_CLOCKS	49
/* iCE40 Programming and Configuration (FPGA-TN-02001 3.1), Appendix A,
 * Table A.1 "iCE40 SRAM Configuration Sequence"
 */
#define ICE40_SS_DUMMY_CLOCKS	8	/* step 2 */
#define ICE40_POST_CLOCKS		100	/* step 5 */
/* the image is sent, and its clock rate checked, ICE40_RATE_CHUNK bytes
 * at a time: each chunk's own rate, and the mean over the whole image
 */
#define ICE40_RATE_CHUNK		4096
/* after a failed load: time for the iCE40 to boot from its flash again */
#define ICE40_BOOT_WAIT_MS		3000

/* the load's signal handler sets this to the signal number */
static volatile sig_atomic_t ice40_slave_signal = 0;

static void ice40_slave_on_signal(int sig)
{
	ice40_slave_signal = sig;
}

/* SIGINT, SIGTERM, SIGHUP, SIGQUIT and SIGPIPE stop the load, at the next
 * bit, instead of killing the process with the iCE40 left unconfigured in
 * slave mode: the load then gives the bus back and resets the iCE40 to boot
 * from its flash.
 * Installed for the duration of the load; the previous handlers are put
 * back afterwards. SIGKILL cannot be caught, and a crash
 * (SIGSEGV, SIGBUS, SIGABRT) is not handled.
 */
class Ice40SlaveSignals {
	public:
		Ice40SlaveSignals() {
			ice40_slave_signal = 0;
			struct sigaction sa;
			memset(&sa, 0, sizeof(sa));
			sa.sa_handler = ice40_slave_on_signal;
			sigemptyset(&sa.sa_mask);
			for (_n = 0; _n < 5; _n++) {
				if (sigaction(sigs()[_n], &sa, &_old[_n]) != 0) {
					int err = errno;
					restore();
					throw std::runtime_error(std::string("cannot handle ") +
						Ice40SpiSlaveAborted::name(sigs()[_n]) + ": " +
						strerror(err) + "; nothing was driven");
				}
			}
		}
		~Ice40SlaveSignals() {
			restore();
		}
	private:
		static const int *sigs() {
			static const int s[5] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT,
				SIGPIPE};
			return s;
		}
		void restore() {
			for (int i = 0; i < _n; i++)
				if (sigaction(sigs()[i], &_old[i], NULL) != 0)
					printError("cannot restore the handler of " +
						Ice40SpiSlaveAborted::name(sigs()[i]) + ": " +
						strerror(errno));
			_n = 0;
		}
		struct sigaction _old[5];
		int _n;
};

/* sleep at least us microseconds, whatever signal arrives meanwhile */
static void sleep_us(long us)
{
	struct timespec t = {us / 1000000, (us % 1000000) * 1000};
	while (nanosleep(&t, &t) != 0) {
		if (errno != EINTR)
			throw std::runtime_error(std::string("nanosleep: ") +
				strerror(errno));
	}
}

static std::string mhz(double hz)
{
	std::ostringstream ss;
	ss.precision(3);
	ss << std::fixed << hz / 1e6 << " MHz";
	return ss.str();
}

/* the SPI_SCK rate of `what` must be inside the data sheet's Slave SPI
 * write range (TN-02001 13.2: "Once the AP sends the 0x7EAA997E
 * synchronization pattern, the generated SPI_SCK clock frequency must be
 * within the range specified in the data sheet"; Table A.1 step 4:
 * "Important: Continuous clock is required")
 */
static double check_rate(const std::string &what, uint32_t bytes,
		std::chrono::steady_clock::duration elapsed)
{
	double hz = (bytes * 8.0) /
		std::chrono::duration<double>(elapsed).count();
	if (hz < ICE40_SLAVE_SCK_MIN_HZ)
		throw std::runtime_error("SPI_SCK " + mhz(hz) + " over " + what +
			": below the iCE40's 1 MHz minimum for a slave SPI configuration "
			"(DS-02008 Table 4.24); this cable is too slow on this host, or "
			"was held up, load stopped");
	if (hz > ICE40_SLAVE_SCK_MAX_HZ)
		throw std::runtime_error("SPI_SCK " + mhz(hz) + " over " + what +
			": above the iCE40's 25 MHz maximum for a slave SPI configuration "
			"(DS-02008 Table 4.24), load stopped");
	return hz;
}

/* CRESET_B let go when no bus line was driven: says whether it was */
std::string Ice40::let_go_reset()
{
	try {
		_slave->hold_reset(false);
	} catch (std::exception &e) {
		return std::string("CRESET_B could not be let go: ") + e.what();
	}
	return "CRESET_B let go, no SPI line was driven";
}

/* SRAM load as an SPI slave over any Ice40SpiSlaveInterface: TN-02001
 * section 13.2 and Appendix A Table A.1, timings from DS-02008 Table 4.24.
 * Throws std::runtime_error, saying what was done to give the board back,
 * on any failure, and when SIGINT, SIGTERM, SIGHUP, SIGQUIT or SIGPIPE stops it.
 */
void Ice40::program_cram_slave(const uint8_t *data, uint32_t length)
{
	Ice40SlaveSignals signals;
	struct AbortFlag {
		explicit AbortFlag(Ice40SpiSlaveInterface *s): slave(s) {
			slave->set_abort(&ice40_slave_signal);
		}
		~AbortFlag() { slave->set_abort(NULL); }
		Ice40SpiSlaveInterface *slave;
	} abort_flag(_slave);

	/* the backend settles what needs no line (its clock, its limits)
	 * before any pin is driven or changed: a host that can never load stops
	 * here with the running design untouched
	 */
	_slave->prepare();

	/* CDONE must be high first: an iCE40 holding a design. If it is
	 * already low, CDONE low under CRESET_B proves nothing, and the SPI
	 * lines are not driven.
	 */
	if (!_slave->cdone())
		throw std::runtime_error("CDONE is low before the reset: no configured "
			"iCE40 is seen on these lines, so nothing was driven");
	_slave->abort_check();

	/* Table A.1 step 1: CRESET_B low for at least tCRESET_B; CDONE must
	 * fall, or no iCE40 is held on that reset and the bus may be in use.
	 */
	_slave->hold_reset(true);
	bool cdone_in_reset;
	try {
		sleep_us(ICE40_CRESET_LOW_US);
		cdone_in_reset = _slave->cdone();
	} catch (std::exception &e) {
		throw std::runtime_error(std::string(e.what()) + "; " + let_go_reset());
	}
	if (cdone_in_reset)
		throw std::runtime_error("CDONE stayed high with CRESET_B held low: "
			"no iCE40 is held on that reset; " + let_go_reset());

	bool slave_mode = false;  // CRESET_B let go with SPI_SS low
	try {
		_slave->abort_check();
		/* the iCE40 is in reset (its SPI pins released): the bus is ours */
		_slave->take_bus();
		_slave->set_ss(false);
		sleep_us(ICE40_CRESET_LOW_US);
		_slave->abort_check();
		/* "The iCE40 FPGA enters SPI peripheral mode when the CRESET_B pin
		 * returns High while the SPI_SS pin is Low" (13.2)
		 */
		_slave->hold_reset(false);
		slave_mode = true;
		sleep_us(ICE40_CR_SCK_US);

		/* step 2: "toggle SPI_SS to high, send 8 dummy clocks, then
		 * toggle it back to low"
		 */
		_slave->set_ss(true);
		_slave->clocks(ICE40_SS_DUMMY_CLOCKS);
		_slave->set_ss(false);

		/* steps 3 and 4: the image, msb first, SPI_SS held low ("Hold
		 * previous SPI_SS state, no toggle"), a continuous clock: every
		 * chunk's own rate is checked, so a stall is caught as well as a
		 * slow cable
		 */
		ProgressBar progress("Loading to CRAM", length, 50, _quiet);
		std::chrono::steady_clock::time_point start =
			std::chrono::steady_clock::now();
		std::chrono::steady_clock::time_point t0 = start;
		uint32_t size;
		for (uint32_t addr = 0; addr < length; addr += size) {
			size = (addr + ICE40_RATE_CHUNK > length) ?
				(length - addr) : ICE40_RATE_CHUNK;
			_slave->write(data + addr, size);
			std::chrono::steady_clock::time_point t1 =
				std::chrono::steady_clock::now();
			check_rate("bytes " + std::to_string(addr) + "-" +
				std::to_string(addr + size - 1) + " of the image", size,
				t1 - t0);
			t0 = t1;
			progress.display(addr + size);
		}
		double hz = check_rate("the image", length,
			std::chrono::steady_clock::now() - start);
		progress.done();
		printInfo("SPI_SCK " + mhz(hz) + " (mean over the image)");

		/* step 5: 100 clocks; step 6: CDONE must be high. SPI_SS stays
		 * low: Table A.1 does not toggle it (Figure 13.2 draws it high
		 * here; openFPGALoader's FTDI load, iceprog and fomu-flash keep it
		 * low, as Table A.1 reads)
		 */
		_slave->clocks(ICE40_POST_CLOCKS);
		if (!_slave->cdone())
			throw std::runtime_error("CDONE stayed low after the image and " +
				std::to_string(ICE40_POST_CLOCKS) + " clocks: the iCE40 did not "
				"accept the configuration");
		/* step 7: at least 49 more clocks (tDONE_IO), then the SPI pins
		 * are the design's I/O: the bus is given back at once
		 */
		_slave->clocks(ICE40_DONE_IO_CLOCKS);
		_slave->set_ss(true);
		_slave->release_bus();
	} catch (std::exception &e) {
		/* a second signal now must not stop the recovery */
		_slave->set_abort(NULL);
		throw std::runtime_error(std::string(e.what()) + "; " +
			recover_slave(slave_mode));
	}
	if (ice40_slave_signal)
		throw std::runtime_error("interrupted by " +
			Ice40SpiSlaveAborted::name(ice40_slave_signal) +
			" after the design was loaded (CDONE high), bus released");
	printSuccess("CDONE high: design loaded");
}

/* after a failure: the bus back to inputs; then CRESET_B let go only if
 * none of the bus lines is still driven, so the iCE40 masters its flash
 * alone when it boots. If the iCE40 had already left reset (slave mode),
 * it is reset again so that it boots from its flash, as at power-up.
 * Says what it did; never throws.
 */
std::string Ice40::recover_slave(bool slave_mode)
{
	try {
		_slave->release_bus();
	} catch (std::exception &e) {
		return std::string("the SPI lines could not all be set back to "
			"inputs (") + e.what() + "): " + (slave_mode ?
			"CRESET_B had already been let go, the iCE40 is not reset again" :
			"CRESET_B is left held low until the lines are given back");
	}
	try {
		if (slave_mode) {
			_slave->hold_reset(true);
			sleep_us(ICE40_CRESET_LOW_US);
		}
		_slave->hold_reset(false);
		for (int ms = 0; ms < ICE40_BOOT_WAIT_MS; ms += 10) {
			if (_slave->cdone())
				return "SPI lines released; the iCE40 was reset and booted "
					"from its flash again (CDONE high)";
			sleep_us(10000);
		}
	} catch (std::exception &e) {
		return std::string("SPI lines released; resetting the iCE40 to boot "
			"from its flash failed: ") + e.what();
	}
	return "SPI lines released; the iCE40 was reset but CDONE did not rise "
		"within " + std::to_string(ICE40_BOOT_WAIT_MS / 1000) +
		" s: it did not boot from its flash";
}

void Ice40::program_slave()
{
	if (_file_extension.empty())
		throw std::runtime_error("no bitstream file");

	RawParser bit(_filename, false);

	printInfo("Parse file ", false);
	if (bit.parse() != EXIT_SUCCESS) {
		printError("FAIL");
		throw std::runtime_error("unable to parse '" + _filename + "'");
	}
	printSuccess("DONE");

	uint32_t length = bit.getLength() / 8;
	if (length == 0)
		throw std::runtime_error("'" + _filename + "' is empty");

	program_cram_slave(bit.getData(), length);
}

void Ice40::program(unsigned int offset, bool unprotect_flash)
{
	if (_slave) {
		program_slave();
		return;
	}

	if (_file_extension.empty())
		return;

	RawParser bit(_filename, false);

	printInfo("Parse file ", false);
	if (bit.parse() == EXIT_SUCCESS) {
		printSuccess("DONE");
	} else {
		printError("FAIL");
		return;
	}

	const uint8_t *data = bit.getData();
	int length = bit.getLength() / 8;

	if (_mode == Device::MEM_MODE) {
		program_cram(data, length);
		return;
	}

	_spi->gpio_clear(_rst_pin);

	SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), unprotect_flash,
			_verbose_level);

	flash.erase_and_prog(offset, data, length);

	if (_verify)
		flash.verify(offset, data, length);

	/* release SPI access / reload */
	post_flash_access();
}

bool Ice40::detect_flash(bool full_info)
{
	if (!_spi)
		return flash_access_unsupported();
	/* prepare SPI access */
	prepare_flash_access();
	printInfo("Read Flash ", false);
	try {
		SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), false, _verbose_level);
		flash.read_id();
		if (full_info)
			flash.display_info();
		flash.display_status_reg();
	} catch (std::exception &e) {
		printError("Fail");
		printError(std::string(e.what()));
		return false;
	}

	/* release SPI access / reload */
	return post_flash_access();
}

bool Ice40::dumpFlash(uint32_t base_addr, uint32_t len)
{
	if (!_spi)
		return flash_access_unsupported();
	/* prepare SPI access */
	prepare_flash_access();
	printInfo("Read Flash ", false);
	try {
		SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), false, _verbose_level);
		flash.reset();
		flash.power_up();
		flash.dump(_filename, base_addr, len);
	} catch (std::exception &e) {
		printError("Fail");
		printError(std::string(e.what()));
		return false;
	}

	/* release SPI access / reload */
	return post_flash_access();
}

bool Ice40::protect_flash(uint32_t len)
{
	if (!_spi)
		return flash_access_unsupported();
	/* SPI access */
	prepare_flash_access();
	/* acess */
	try {
		SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), false, _verbose_level);
		/* configure flash protection */
		if (flash.enable_protection(len) == -1)
			return false;
	} catch (std::exception &e) {
		printError("Fail");
		printError(std::string(e.what()));
		return false;
	}

	/* reload */
	return post_flash_access();
}

bool Ice40::unprotect_flash()
{
	if (!_spi)
		return flash_access_unsupported();
	/* SPI access */
	prepare_flash_access();
	/* acess */
	try {
		SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), false, _verbose_level);
		/* configure flash protection */
		if (flash.disable_protection() == -1)
			return false;
	} catch (std::exception &e) {
		printError("Fail");
		printError(std::string(e.what()));
		return false;
	}

	/* reload */
	return post_flash_access();
}

bool Ice40::bulk_erase_flash()
{
	if (!_spi)
		return flash_access_unsupported();
	/* SPI access */
	prepare_flash_access();
	/* acess */
	try {
		SPIFlash flash(reinterpret_cast<SPIInterface *>(_spi), false, _verbose_level);
		/* bulk erase flash */
		if (flash.bulk_erase() == -1)
			return false;
	} catch (std::exception &e) {
		printError("Fail");
		printError(std::string(e.what()));
		return false;
	}

	/* reload */
	return post_flash_access();
}

bool Ice40::prepare_flash_access()
{
	if (!_spi)
		return flash_access_unsupported();
	/* SPI access: shutdown ICE40 */
	_spi->gpio_clear(_rst_pin);
	usleep(1000);
	return true;
}

bool Ice40::post_flash_access()
{
	if (!_spi)
		return flash_access_unsupported();
	reset();
	return ((_spi->gpio_get(true) & _done_pin) == 0) ? false : true;
}
