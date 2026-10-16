// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#include "gpiomemIce40Spi.hpp"

#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <pwd.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iterator>
#include <csignal>
#include <cstdio>
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

#include "display.hpp"

/* The GPIO register block of the BCM2835 (Raspberry Pi 1, Zero), BCM2836
 * (Pi 2), BCM2837 (Pi 3, Zero 2) and BCM2711 (Pi 4), offsets from the
 * block's base, all accesses 32-bit:
 *
 *  - BCM2835 ARM Peripherals (06 February 2012), section 6.1 "Register
 *    View" (bus addresses 0x7E200000 + offset) and the register
 *    descriptions after it: GPFSEL0 .. GPFSEL5 0x00 .. 0x14 ("GPIO
 *    Function Select"), GPSET0 0x1C, GPCLR0 0x28, GPLEV0 0x34;
 *    "FSELn" is 3 bits per pin, 10 pins a register: "000 = GPIO Pin 9 is
 *    an input", "001 = GPIO Pin 9 is an output", 100 .. 010 the alternate
 *    functions; GPSET0 / GPCLR0 "SETn / CLRn (n=0..31) 0 = No effect,
 *    1 = Set / Clear GPIO pin n"; GPLEV0 "LEVn (n=0..31)", the pin's level.
 *  - BCM2711 ARM Peripherals, section 5.2 "Register View", Table 63 "GPIO
 *    Register Assignment": 0x00 GPFSEL0 .. 0x14 GPFSEL5, 0x1c GPSET0,
 *    0x28 GPCLR0, 0x34 GPLEV0; the same FSEL coding (Table 64); "The GPIO
 *    register base address is 0x7e200000".
 *
 * Same pin, same offsets in both. Only the "0" registers (GPIO 0-31) are
 * used. The pull-up/down registers are not touched (the BCM2711 has new
 * ones, the older SoCs have GPPUD; this load leaves every pull as it was).
 */
#define GPIO_GPFSEL0	0x00
#define GPIO_GPSET0		0x1C
#define GPIO_GPCLR0		0x28
#define GPIO_GPLEV0		0x34
#define GPIO_FSEL_INPUT		0u	/* 000 */
#define GPIO_FSEL_OUTPUT	1u	/* 001 */
#define GPIO_FSEL_MASK		7u
#define GPIO_PINS			32	/* GPSET0, GPCLR0, GPLEV0 */
#define GPIO_PAGE			4096	/* /dev/gpiomem maps one page */

/* the SPI_SCK period is held at 50 ns or more (20 MHz), each phase at
 * 25 ns or more: inside the data sheet's 25 MHz maximum and its 20 ns
 * minimum SPI_SCK high and low times (DS-02008 Table 4.24 fMAX, tCCLKH,
 * tCCLKL), with margin
 */
#define SCK_PERIOD_NS	50.0
#define SCK_PHASE_NS	25.0

GpiomemRegs::GpiomemRegs(const std::string &path): _page(MAP_FAILED)
{
	int fd = open(path.c_str(), O_RDWR | O_SYNC | O_CLOEXEC);
	if (fd < 0)
		throw std::runtime_error("cannot open " + path + " read/write: " +
			strerror(errno));
	_page = mmap(NULL, GPIO_PAGE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	int err = errno;
	close(fd);
	if (_page == MAP_FAILED)
		throw std::runtime_error("cannot map " + path + ": " + strerror(err));
}

GpiomemRegs::~GpiomemRegs()
{
	if (_page != MAP_FAILED)
		munmap(_page, GPIO_PAGE);
}

static unsigned int fsel_reg(unsigned int pin)
{
	return GPIO_GPFSEL0 + 4 * (pin / 10);
}

static unsigned int fsel_shift(unsigned int pin)
{
	return 3 * (pin % 10);
}

GpiomemIce40Spi::GpiomemIce40Spi(const ice40_gpio_pins_t &pins,
		std::unique_ptr<Ice40GpioRegs> regs, int8_t verbose):
	Ice40GpioSpi(pins, verbose, GPIO_PINS), _regs(std::move(regs)),
	_n_saved(0), _pad_spins(0)
{
	if (!_regs)
		throw std::logic_error("no GPIO registers");
	/* a pin is driven only if it is a plain GPIO: input or output.
	 * An alternate function belongs to something (SPI0 on GPIO8-11, say),
	 * and nothing here would stop us taking its pin from it.
	 */
	for (unsigned int pin : _lines) {
		uint32_t fn = (_regs->read(fsel_reg(pin)) >> fsel_shift(pin)) &
			GPIO_FSEL_MASK;
		if (fn != GPIO_FSEL_INPUT && fn != GPIO_FSEL_OUTPUT)
			throw std::runtime_error(gpio_name(pin) + " is in alternate "
				"function " + std::to_string(fn) + " (GPFSEL bits): a driver "
				"such as SPI0 may own it, and /dev/gpiomem bypasses the "
				"kernel's pin ownership, so it is not taken over");
		_saved[_n_saved++] = Saved{pin, fn, false};
	}
}

GpiomemIce40Spi::~GpiomemIce40Spi()
{
	std::vector<std::string> stuck;
	try {
		stuck = release_all();
	} catch (std::exception &e) {
		stuck.push_back(e.what());
	}
	int bad = restore_functions();
	if (bad)
		stuck.push_back(std::to_string(bad) + " pin(s) did not read back "
			"as their saved function");
	for (const std::string &s : stuck)
		printError("gpiomem iCE40 SPI: " + s);
}

/* Async-signal-safe: register reads and writes, a fixed array, no
 * allocation, no exception, no libc call. Bus lines first, CRESET_B last,
 * the order the lines were saved in.
 */
int GpiomemIce40Spi::restore_functions() noexcept
{
	int bad = 0;
	for (unsigned int i = 0; i < _n_saved; i++) {
		if (!_saved[i].changed)
			continue;
		unsigned int reg = fsel_reg(_saved[i].pin);
		unsigned int shift = fsel_shift(_saved[i].pin);
		_regs->write(reg, (_regs->read(reg) & ~(GPIO_FSEL_MASK << shift)) |
			(_saved[i].function << shift));
		if (((_regs->read(reg) >> shift) & GPIO_FSEL_MASK) !=
				_saved[i].function)
			bad++;
		else
			_saved[i].changed = false;
	}
	return bad;
}

void GpiomemIce40Spi::mark_changed(unsigned int offset)
{
	for (unsigned int i = 0; i < _n_saved; i++)
		if (_saved[i].pin == offset)
			_saved[i].changed = true;
}

/* read-modify-write of the pin's three FSEL bits */
void GpiomemIce40Spi::fsel(unsigned int offset, uint32_t function)
{
	unsigned int reg = fsel_reg(offset);
	unsigned int shift = fsel_shift(offset);
	mark_changed(offset);
	_regs->write(reg, (_regs->read(reg) & ~(GPIO_FSEL_MASK << shift)) |
		(function << shift));
	uint32_t now = (_regs->read(reg) >> shift) & GPIO_FSEL_MASK;
	if (now != function)
		throw std::runtime_error("unable to change the function of " +
			gpio_name(offset) + ": wrote " + std::to_string(function) +
			", read back " + std::to_string(now));
}

/* the level first, then the pin becomes an output: it comes up at that
 * level ("if the pin is subsequently defined as an output then the bit
 * will be set according to the last set/clear operation", BCM2835 section
 * 6.2 GPSETn, BCM2711 GPSET0)
 */
void GpiomemIce40Spi::output(unsigned int offset, bool level)
{
	_regs->write(level ? GPIO_GPSET0 : GPIO_GPCLR0, 1u << offset);
	fsel(offset, GPIO_FSEL_OUTPUT);
	_outputs[offset] = level;
}

void GpiomemIce40Spi::input(unsigned int offset)
{
	if (!_outputs.count(offset))
		return;
	fsel(offset, GPIO_FSEL_INPUT);
	_outputs.erase(offset);
}

void GpiomemIce40Spi::set(unsigned int offset, bool level)
{
	if (!_outputs.count(offset))
		throw std::runtime_error(gpio_name(offset) + " is not driven");
	_regs->write(level ? GPIO_GPSET0 : GPIO_GPCLR0, 1u << offset);
	_outputs[offset] = level;
}

bool GpiomemIce40Spi::get(unsigned int offset)
{
	return (_regs->read(GPIO_GPLEV0) >> offset) & 1;
}

static inline void spin(uint32_t n)
{
	for (uint32_t i = 0; i < n; i++)
		__asm__ __volatile__("" ::: "memory");
}

static double seconds_since(std::chrono::steady_clock::time_point t0)
{
	return std::chrono::duration<double>(
		std::chrono::steady_clock::now() - t0).count();
}

/* nbits bits, mode 0, msb first (data NULL: all ones), SPI_SCK low on
 * entry and on exit. Per bit: the SPI_SI level (SPI_SCK is low), SPI_SCK
 * high, SPI_SCK low: three register writes, with `pad` spins after the
 * first two. SPI_SI changes while SPI_SCK is low and is captured on the
 * rising edge (TN-02001 13.2). Writes of a zero mask change nothing, so
 * the same loop with both masks 0 times the pin writes without moving a
 * pin (calibrate()). `stop` is looked at before every bit, SPI_SCK being
 * low: once it is set the loop returns how many bits were sent.
 */
static uint32_t shift_bits(Ice40GpioRegs *regs,
		const volatile sig_atomic_t *stop, const uint8_t *data,
		uint32_t nbits, uint32_t sck_mask, uint32_t si_mask, uint32_t pad)
{
	for (uint32_t i = 0; i < nbits; i++) {
		if (*stop)
			return i;
		bool bit = data ? (data[i >> 3] >> (7 - (i & 7))) & 1 : true;
		regs->write(bit ? GPIO_GPSET0 : GPIO_GPCLR0, si_mask);
		spin(pad);
		regs->write(GPIO_GPSET0, sck_mask);
		spin(pad);
		regs->write(GPIO_GPCLR0, sck_mask);
	}
	return nbits;
}

/* How long a bit takes on this host right now, and how long a spin: the
 * padding that keeps every SPI_SCK period at 50 ns or more and every phase
 * at 25 ns or more (the three writes of a bit: SPI_SCK low for two of
 * them, high for one, each taking w).
 *   high = w + pad >= 25 ns; low = 2w + pad >= 25 ns; 3w + 2 pad >= 50 ns.
 * Measured with zero masks, the shortest of three runs (the least
 * disturbed), before every write(): the CPU clock moves, and so does what
 * a spin costs. Fails when the clock gives no usable time.
 */
void GpiomemIce40Spi::calibrate()
{
	static const uint8_t pattern[32] = {
		0xA5, 0x5A, 0xFF, 0x00, 0xA5, 0x5A, 0xFF, 0x00,
		0xA5, 0x5A, 0xFF, 0x00, 0xA5, 0x5A, 0xFF, 0x00,
		0xA5, 0x5A, 0xFF, 0x00, 0xA5, 0x5A, 0xFF, 0x00,
		0xA5, 0x5A, 0xFF, 0x00, 0xA5, 0x5A, 0xFF, 0x00};
	static const volatile sig_atomic_t no_stop = 0;
	const uint32_t nbits = sizeof(pattern) * 8;
	const uint32_t nspins = 20000;
	double bit_s = 0, spin_s = 0;
	for (int run = 0; run < 3; run++) {
		auto t0 = std::chrono::steady_clock::now();
		shift_bits(_regs.get(), &no_stop, pattern, nbits, 0, 0, 0);
		double b = seconds_since(t0) / nbits;
		t0 = std::chrono::steady_clock::now();
		spin(nspins);
		double s = seconds_since(t0) / nspins;
		if (run == 0 || b < bit_s)
			bit_s = b;
		if (run == 0 || s < spin_s)
			spin_s = s;
	}
	if (!(bit_s > 0) || !(spin_s > 0))
		throw std::runtime_error("the clock cannot time a register write "
			"(bit " + std::to_string(bit_s * 1e9) + " ns, spin " +
			std::to_string(spin_s * 1e9) + " ns): SPI_SCK cannot be held "
			"to the iCE40's 25 MHz maximum, nothing sent");
	_pad_spins = pad_spins_for(bit_s * 1e9, spin_s * 1e9);
}

uint32_t GpiomemIce40Spi::pad_spins_for(double bit_ns, double spin_ns)
{
	double w = bit_ns / 3;
	double pad_ns = std::max(0.0, std::max(SCK_PHASE_NS - w,
		(SCK_PERIOD_NS - 3 * w) / 2));
	return static_cast<uint32_t>(std::ceil(pad_ns / spin_ns));
}

void GpiomemIce40Spi::write(const uint8_t *data, uint32_t len)
{
	if (!_outputs.count(_pins.sck) || !_outputs.count(_pins.si))
		throw std::runtime_error("SPI_SCK and SPI_SI are not driven");
	calibrate();
	uint32_t sent = shift_bits(_regs.get(), abort_flag(), data,
		len * 8, 1u << _pins.sck, 1u << _pins.si, _pad_spins);
	/* SPI_SCK is low at every bit boundary; SPI_SI is the last bit sent */
	_outputs[_pins.sck] = false;
	if (sent)
		_outputs[_pins.si] = (data[(sent - 1) >> 3] >>
			(7 - ((sent - 1) & 7))) & 1;
	if (sent < len * 8) {
		abort_check();
		throw std::logic_error("the bit loop stopped without a stop request");
	}
}

void GpiomemIce40Spi::clocks(uint32_t nclk)
{
	set(_pins.si, true);
	calibrate();
	uint32_t sent = shift_bits(_regs.get(), abort_flag(), NULL, nclk,
		1u << _pins.sck, 1u << _pins.si, _pad_spins);
	if (sent < nclk) {
		abort_check();
		throw std::logic_error("the bit loop stopped without a stop request");
	}
}

static std::string group_name(gid_t gid)
{
	struct group *g = getgrgid(gid);
	return g ? g->gr_name : "gid " + std::to_string(gid);
}

/* who this is and what the device allows: said, never changed */
static std::string access_note(const std::string &path)
{
	std::string who;
	struct passwd *pw = getpwuid(geteuid());
	who = "running as uid " + std::to_string(geteuid()) +
		(pw ? std::string(" (") + pw->pw_name + ")" : std::string()) +
		", groups: ";
	int n = getgroups(0, NULL);
	std::vector<gid_t> gids(n > 0 ? n : 0);
	if (n > 0)
		n = getgroups(n, gids.data());
	if (n < 0)
		who += std::string("unreadable (") + strerror(errno) + ")";
	for (int i = 0; i < n; i++)
		who += (i ? ", " : "") + group_name(gids[i]);
	struct stat st;
	if (stat(path.c_str(), &st) == 0) {
		char mode[8];
		snprintf(mode, sizeof(mode), "%04o", st.st_mode & 07777);
		struct passwd *o = getpwuid(st.st_uid);
		who += "; " + path + " is " + (o ? o->pw_name : "uid " +
			std::to_string(st.st_uid)) + ":" + group_name(st.st_gid) +
			" mode " + mode;
	}
	return who;
}

std::unique_ptr<Ice40GpioRegs> GpiomemIce40Spi::open_gpiomem(
		const std::string &path)
{
	/* a BCM283x host: libgpiod is not an answer to a missing or closed
	 * /dev/gpiomem, so the load stops here and says why
	 */
	const std::string end = ": the load stops here, libgpiod is not used "
		"instead (it was measured under the iCE40's 1 MHz minimum SPI_SCK on a "
		"Pi 3 Model B+)";
	if (access(path.c_str(), F_OK) != 0) {
		int err = errno;
		throw std::runtime_error("this host is a BCM283x/BCM2711 Raspberry "
			"Pi, whose GPIO registers are reached through " + path +
			", but " + path + " is absent (" + strerror(err) + "): is the "
			"bcm2835-gpiomem driver loaded?" + end);
	}
	try {
		return std::unique_ptr<Ice40GpioRegs>(new GpiomemRegs(path));
	} catch (std::exception &e) {
		throw std::runtime_error("this host is a BCM283x/BCM2711 Raspberry "
			"Pi, whose GPIO registers are reached through " + path + ", but " +
			e.what() + ": is the user in the gpio group? (" +
			access_note(path) + ")" + end);
	}
}

GpiomemIce40Spi::Choice GpiomemIce40Spi::select(bool chip_given,
		const std::string &compatible)
{
	if (chip_given)
		return Choice{false, "a gpiochip was named with -d, so libgpiod "
			"drives the lines"};
	std::ifstream f(compatible, std::ios::binary);
	if (!f)
		return Choice{false, "cannot read " + compatible + " (" +
			strerror(errno) + "): not known to be a BCM283x/BCM2711 "
			"Raspberry Pi, so libgpiod drives the lines"};
	std::string all((std::istreambuf_iterator<char>(f)),
		std::istreambuf_iterator<char>());
	static const char *socs[] = {"brcm,bcm2835", "brcm,bcm2836",
		"brcm,bcm2837", "brcm,bcm2711"};
	std::string list;
	size_t pos = 0;
	while (pos < all.size()) {
		std::string entry(all.c_str() + pos);
		pos += entry.size() + 1;
		list += (list.empty() ? "" : ", ") + entry;
		for (const char *soc : socs)
			if (entry == soc)
				return Choice{true, compatible + " lists " + entry +
					": a Raspberry Pi 1-4, its GPIO registers are driven "
					"through /dev/gpiomem (libgpiod was measured under the "
					"iCE40's 1 MHz minimum SPI_SCK on a Pi 3 Model B+)"};
	}
	return Choice{false, compatible + " lists \"" + list + "\", no "
		"brcm,bcm2835/2836/2837/2711: not a Raspberry Pi 1-4, so libgpiod "
		"drives the lines"};
}
