// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2025 Tim Ansell <me@mith.ro>
 *
 * TinyTapeout FPGA Demo Board support via MicroPython raw REPL.
 * Programs iCE40UP5K FPGA through RP2040/RP2350 running MicroPython.
 */

#include "ttMicropython.hpp"

#if defined(_WIN32) || defined(_WIN64)
#include <setupapi.h>
#include <devguid.h>
#include <cfgmgr32.h>
#pragma comment(lib, "setupapi.lib")
#else
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#endif

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "display.hpp"
#include "progressBar.hpp"

/* Cross-platform millisecond sleep */
static void sleep_ms(int ms)
{
#if defined(_WIN32) || defined(_WIN64)
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

/* ─── Base64 encoding table ─── */
static const char b64_table[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string base64_encode(const uint8_t *data, size_t len)
{
	std::string out;
	out.reserve(((len + 2) / 3) * 4);
	for (size_t i = 0; i < len; i += 3) {
		uint32_t n = static_cast<uint32_t>(data[i]) << 16;
		if (i + 1 < len) n |= static_cast<uint32_t>(data[i + 1]) << 8;
		if (i + 2 < len) n |= static_cast<uint32_t>(data[i + 2]);
		out.push_back(b64_table[(n >> 18) & 0x3F]);
		out.push_back(b64_table[(n >> 12) & 0x3F]);
		out.push_back((i + 1 < len) ? b64_table[(n >> 6) & 0x3F] : '=');
		out.push_back((i + 2 < len) ? b64_table[n & 0x3F] : '=');
	}
	return out;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Constructor / Destructor
 * ═══════════════════════════════════════════════════════════════════════════ */

TTMicropython::TTMicropython(const std::string &filename,
                             const std::string &file_type,
                             const std::string &serial_port,
                             bool write_flash, bool verify, int8_t verbose)
    :
#if defined(_WIN32) || defined(_WIN64)
      _serial_handle(INVALID_HANDLE_VALUE),
#else
      _fd(-1),
#endif
      _filename(filename),
      _file_type(file_type),
      _serial_port(serial_port),
      _write_flash(write_flash),
      _verify(verify),
      _verbose(verbose),
      _in_raw_repl(false)
{
}

TTMicropython::~TTMicropython()
{
	if (_in_raw_repl) {
		try { exitRawRepl(); } catch (...) {}
	}
	closeSerial();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Serial port – platform abstraction
 * ═══════════════════════════════════════════════════════════════════════════ */

#if defined(_WIN32) || defined(_WIN64)

/* ── Windows serial implementation ── */

void TTMicropython::openSerial(const std::string &port)
{
	/* Windows COM ports above COM9 need the \\.\COMx prefix */
	std::string dev = port;
	if (dev.find("\\\\.\\") != 0 && dev.find("COM") == 0)
		dev = "\\\\.\\" + dev;

	HANDLE h = CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE,
		0, NULL, OPEN_EXISTING, 0, NULL);
	if (h == INVALID_HANDLE_VALUE) {
		DWORD err = GetLastError();
		std::string hint;
		if (err == ERROR_FILE_NOT_FOUND)
			hint = " (device not found — is the board connected?)";
		else if (err == ERROR_ACCESS_DENIED)
			hint = " (access denied — another process may be using "
			       "this port)";
		throw std::runtime_error("Cannot open serial port " + port +
		                         " (error " + std::to_string(err) +
		                         ")" + hint);
	}

	/* Configure 115200 8N1, no flow control */
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (!GetCommState(h, &dcb)) {
		CloseHandle(h);
		throw std::runtime_error("GetCommState failed");
	}
	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fBinary = TRUE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	if (!SetCommState(h, &dcb)) {
		CloseHandle(h);
		throw std::runtime_error("SetCommState failed");
	}

	/* Set timeouts: reads return immediately with whatever is available */
	COMMTIMEOUTS timeouts;
	memset(&timeouts, 0, sizeof(timeouts));
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 5000;
	SetCommTimeouts(h, &timeouts);

	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

	_serial_handle = h;
}

void TTMicropython::closeSerial()
{
	if (_serial_handle != INVALID_HANDLE_VALUE) {
		CloseHandle(_serial_handle);
		_serial_handle = INVALID_HANDLE_VALUE;
	}
}

int TTMicropython::serialWrite(const uint8_t *data, size_t len)
{
	DWORD written = 0;
	if (!WriteFile(_serial_handle, data, static_cast<DWORD>(len),
	               &written, NULL))
		return -1;
	return static_cast<int>(written);
}

int TTMicropython::serialRead(uint8_t *buf, size_t maxlen,
                              int timeout_ms)
{
	/* Set read timeout for this call */
	COMMTIMEOUTS timeouts;
	memset(&timeouts, 0, sizeof(timeouts));
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
	timeouts.ReadTotalTimeoutConstant = timeout_ms;
	SetCommTimeouts(_serial_handle, &timeouts);

	DWORD bytes_read = 0;
	if (!ReadFile(_serial_handle, buf, static_cast<DWORD>(maxlen),
	              &bytes_read, NULL))
		return -1;
	return static_cast<int>(bytes_read);
}

#else /* POSIX (Linux + macOS) */

void TTMicropython::openSerial(const std::string &port)
{
	int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		std::string hint;
		if (errno == EACCES)
			hint = " (permission denied — try running as root "
			       "or add user to 'dialout' group)";
		else if (errno == EBUSY)
			hint = " (device busy — another process may be using "
			       "this port, e.g. mpremote or minicom)";
		else if (errno == ENOENT)
			hint = " (device not found — is the board connected?)";
		throw std::runtime_error("Cannot open serial port " + port +
		                         ": " + strerror(errno) + hint);
	}

	/* Clear O_NONBLOCK after open — we use select() for timeouts */
	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0) {
		close(fd);
		throw std::runtime_error("tcgetattr failed: " +
		                         std::string(strerror(errno)));
	}

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	/* 8N1, no flow control */
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CLOCAL | CREAD;

	/* Raw mode */
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
	                  INLCR | IGNCR | ICRNL);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;  /* 100ms inter-char timeout */

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		close(fd);
		throw std::runtime_error("tcsetattr failed: " +
		                         std::string(strerror(errno)));
	}

	tcflush(fd, TCIOFLUSH);

	_fd = fd;
}

void TTMicropython::closeSerial()
{
	if (_fd >= 0) {
		close(_fd);
		_fd = -1;
	}
}

int TTMicropython::serialWrite(const uint8_t *data, size_t len)
{
	size_t written = 0;
	while (written < len) {
		ssize_t n = write(_fd, data + written, len - written);
		if (n < 0) {
			if (errno == EINTR) continue;
			return -1;
		}
		written += n;
	}
	return static_cast<int>(written);
}

int TTMicropython::serialRead(uint8_t *buf, size_t maxlen,
                              int timeout_ms)
{
	fd_set rfds;
	struct timeval tv;
	FD_ZERO(&rfds);
	FD_SET(_fd, &rfds);
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	int ret = select(_fd + 1, &rfds, NULL, NULL, &tv);
	if (ret < 0 && errno != EINTR) return -1;
	if (ret <= 0) return 0;  /* timeout or EINTR */

	ssize_t n;
	do {
		n = read(_fd, buf, maxlen);
	} while (n < 0 && errno == EINTR);
	return static_cast<int>(n);
}

#endif /* _WIN32 */

void TTMicropython::drainSerial()
{
	uint8_t buf[256];
	while (serialRead(buf, sizeof(buf), 100) > 0) {
		/* discard */
	}
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  MicroPython raw REPL protocol
 * ═══════════════════════════════════════════════════════════════════════════ */

bool TTMicropython::enterRawRepl()
{
	if (_in_raw_repl) return true;

	/* Try up to 3 times to enter raw REPL.
	 * The board may be in various states: running a program, at the
	 * friendly REPL, in paste mode, or even in a crashed state.
	 */
	for (int retry = 0; retry < 3; retry++) {
		if (retry > 0) {
			if (_verbose > 0)
				printInfo("Retrying raw REPL entry (attempt " +
				          std::to_string(retry + 1) + "/3)...");
			sleep_ms(500);  /* 500ms between retries */
		}

		/* Interrupt any running program: Ctrl-C twice */
		const uint8_t ctrl_c[] = {0x03, 0x03};
		serialWrite(ctrl_c, 2);
		sleep_ms(100);  /* 100ms */
		drainSerial();

		/* Enter raw REPL: Ctrl-A */
		const uint8_t ctrl_a = 0x01;
		serialWrite(&ctrl_a, 1);

		/* Wait for prompt: "raw REPL; CTRL-B to exit\r\n>" */
		std::string response;
		uint8_t buf[256];
		int attempts = 0;
		while (attempts < 50) {  /* up to 5 seconds */
			ssize_t n = serialRead(buf, sizeof(buf), 100);
			if (n > 0) {
				response.append(reinterpret_cast<char *>(buf),
				                n);
				size_t rp = response.find("raw REPL");
				if (rp != std::string::npos &&
				    response.find('>', rp) !=
				    std::string::npos) {
					_in_raw_repl = true;
					if (_verbose > 0)
						printInfo("Entered MicroPython "
						          "raw REPL");
					return true;
				}
			}
			attempts++;
		}

		if (_verbose > 0)
			printInfo("Raw REPL not detected, response: " +
			          response.substr(0, 100));
	}

	printError("Failed to enter MicroPython raw REPL after 3 attempts");
	return false;
}

void TTMicropython::exitRawRepl()
{
	if (!_in_raw_repl) return;

	/* Exit raw REPL: Ctrl-B */
	const uint8_t ctrl_b = 0x02;
	serialWrite(&ctrl_b, 1);
	usleep(100000);
	drainSerial();
	_in_raw_repl = false;

	if (_verbose > 0)
		printInfo("Exited MicroPython raw REPL");
}

TTMicropython::ReplResult TTMicropython::execRaw(const std::string &code,
                                                 int timeout_ms)
{
	ReplResult result;
	result.ok = false;

	if (!_in_raw_repl) {
		result.err = "Not in raw REPL mode";
		return result;
	}

	/* Send code + Ctrl-D to execute */
	serialWrite(reinterpret_cast<const uint8_t *>(code.c_str()),
	            code.size());
	const uint8_t ctrl_d = 0x04;
	serialWrite(&ctrl_d, 1);

	/* Read response: OK<stdout>\x04<stderr>\x04> */
	std::string raw;
	uint8_t buf[1024];
	int elapsed = 0;
	const int poll_ms = 50;

	while (elapsed < timeout_ms) {
		ssize_t n = serialRead(buf, sizeof(buf), poll_ms);
		if (n > 0) {
			raw.append(reinterpret_cast<char *>(buf), n);
			/* Check if we have the full response:
			 * "OK" + stdout + \x04 + stderr + \x04 + ">"
			 * We need at least 2 \x04 markers after "OK"
			 */
			size_t ok_pos = raw.find("OK");
			if (ok_pos != std::string::npos) {
				/* Count \x04 markers after OK */
				int markers = 0;
				size_t last_marker = std::string::npos;
				for (size_t i = ok_pos + 2; i < raw.size(); i++) {
					if (raw[i] == 0x04) {
						markers++;
						if (markers == 1)
							last_marker = i;
						else if (markers == 2) {
							last_marker = i;
							break;
						}
					}
				}
				if (markers >= 2 && last_marker + 1 < raw.size()) {
					/* Parse: OK<stdout>\x04<stderr>\x04> */
					size_t stdout_start = ok_pos + 2;
					size_t first_marker = raw.find('\x04',
					                               stdout_start);
					result.out = raw.substr(stdout_start,
					                        first_marker - stdout_start);
					size_t stderr_start = first_marker + 1;
					result.err = raw.substr(stderr_start,
					                        last_marker - stderr_start);
					result.ok = result.err.empty();
					return result;
				}
			}
		}
		elapsed += poll_ms;
	}

	result.err = "Timeout waiting for raw REPL response";
	if (_verbose > 0 && !raw.empty())
		printError("Partial response: " + raw);
	return result;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SRAM Programming
 * ═══════════════════════════════════════════════════════════════════════════ */

bool TTMicropython::setupSramProgramming()
{
	/* Setup script: configure pins, reset FPGA per iCE40 TN1248 protocol.
	 * Pin assignments come from fabricfox.pin_indices():
	 *   sck=0 (RP_PROJCLK), mosi=8 (CINC_UO_OUT3),
	 *   ss=6 (CTRL_ENA_UO_OUT1), reset=7 (nCRST_UO_OUT2)
	 * Hardware SPI cannot use GPIO 0 as SCK, so we always bitbang.
	 *
	 * iCE40 SPI Slave Configuration sequence (TN1248):
	 * 1. Drive SS low
	 * 2. Drive CRESET_B low
	 * 3. Wait >= 200ns
	 * 4. Release CRESET_B (high)
	 * 5. Wait >= 1200us for iCE40 to clear config memory
	 * 6. Send 8 dummy clocks
	 * 7. Send bitstream (CPOL=0, CPHA=0, MSB first, SS stays low)
	 * 8. Send >= 49 dummy clocks
	 * 9. Release SS
	 */
	std::string setup = R"(
from machine import Pin
import time

# TTDBv3 SPI programming pins (hardcoded to bypass GPIOMap firmware bug:
# all boards load GPIOMapTT04 instead of GPIOMapTTDBv3, so pin_indices()
# returns wrong GPIO numbers).
#   sck=6 (MNG03), mosi=3 (MNG00), ss=5 (MNG02), reset=1 (CTRL_SEL_nRST)
sck = Pin(6, Pin.OUT, value=0)
mosi = Pin(3, Pin.OUT, value=0)
ss = Pin(5, Pin.OUT, value=1)
reset = Pin(1, Pin.OUT, value=1)

# iCE40 SPI slave config entry:
# 1. CRESET low, SS low
reset.value(0)
ss.value(0)
time.sleep_ms(15)
# 2. Release CRESET
reset.value(1)
time.sleep_ms(15)

# Dummy clocks with SS HIGH (synchronization per fabricfox DoDummyClocks)
ss.value(1)
time.sleep_ms(2)
for _ in range(8):
    sck.value(1)
    sck.value(0)
time.sleep_us(20)
# Pull SS low for bitstream transfer
ss.value(0)
time.sleep_ms(2)
print("SETUP_OK")
)";

	ReplResult r = execRaw(setup, 5000);
	if (!r.ok || r.out.find("SETUP_OK") == std::string::npos) {
		printError("SRAM setup failed");
		if (!r.err.empty())
			printError("Error: " + r.err);
		if (_verbose > 0 && !r.out.empty())
			printInfo("Output: " + r.out);
		return false;
	}

	if (_verbose > 0)
		printInfo("SRAM programming setup complete");
	return true;
}

bool TTMicropython::sendBitstreamChunk(const uint8_t *data, size_t len)
{
	std::string b64 = base64_encode(data, len);

	std::string code =
		"import ubinascii\n"
		"d = ubinascii.a2b_base64(b'" + b64 + "')\n"
		"for byte in d:\n"
		"    for i in range(7, -1, -1):\n"
		"        mosi.value((byte >> i) & 1)\n"
		"        sck.value(1)\n"
		"        sck.value(0)\n"
		"print('CHUNK_OK')\n";

	ReplResult r = execRaw(code, 10000);
	if (!r.ok || r.out.find("CHUNK_OK") == std::string::npos) {
		printError("Bitstream chunk send failed");
		if (!r.err.empty())
			printError("Error: " + r.err);
		return false;
	}
	return true;
}

bool TTMicropython::finalizeSramProgramming()
{
	std::string code = R"(
# Send >= 49 dummy clocks after bitstream (we send 100)
mosi.value(0)
for _ in range(100):
    sck.value(1)
    sck.value(0)
# Release SS
ss.value(1)
# A few more clocks for good measure
for _ in range(10):
    sck.value(1)
    sck.value(0)
print("DONE_OK")
)";

	ReplResult r = execRaw(code, 5000);
	if (!r.ok || r.out.find("DONE_OK") == std::string::npos) {
		printError("SRAM finalize failed");
		if (!r.err.empty())
			printError("Error: " + r.err);
		return false;
	}
	return true;
}

void TTMicropython::program_sram()
{
	printInfo("TT FPGA: SRAM programming via MicroPython raw REPL");

	/* Read bitstream file */
	std::ifstream file(_filename, std::ios::binary | std::ios::ate);
	if (!file.is_open())
		throw std::runtime_error("Cannot open bitstream file: " + _filename);

	std::streamsize size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<uint8_t> bitstream(size);
	if (!file.read(reinterpret_cast<char *>(bitstream.data()), size))
		throw std::runtime_error("Failed to read bitstream file: " + _filename);
	file.close();

	printInfo("Bitstream: " + _filename + " (" +
	          std::to_string(size) + " bytes)");

	/* Open serial and enter raw REPL */
	openSerial(_serial_port);
	if (!enterRawRepl())
		throw std::runtime_error("Failed to enter raw REPL on " +
		                         _serial_port);

	/* Setup SRAM programming (reset FPGA, init SPI) */
	if (!setupSramProgramming())
		throw std::runtime_error("SRAM programming setup failed");

	/* Stream bitstream in chunks.
	 * If any chunk fails, we must still finalize (release SS, send
	 * dummy clocks) so the iCE40 isn't left in configuration mode.
	 */
	bool sram_ok = true;
	std::string sram_err;
	try {
		const size_t chunk_size = 256;
		size_t total = bitstream.size();
		size_t sent = 0;

		ProgressBar progress("Writing", total, 50, _verbose < 0);
		while (sent < total) {
			size_t n = std::min(chunk_size, total - sent);
			if (!sendBitstreamChunk(bitstream.data() + sent, n)) {
				sram_err = "Failed to send bitstream chunk at "
				           "offset " + std::to_string(sent);
				sram_ok = false;
				break;
			}
			sent += n;
			progress.display(sent);
		}
		progress.done();
	} catch (std::exception &e) {
		sram_err = e.what();
		sram_ok = false;
	}

	/* Always finalize: dummy clocks, release SS */
	finalizeSramProgramming();

	if (!sram_ok)
		throw std::runtime_error(sram_err);

	/* Post-programming: restore board to a usable state.
	 * - Release SPI pins (set to input)
	 * - Start clock on GPIO0 (rp_projclk) for the FPGA design
	 * - Ensure reset is released (high)
	 * - Set all TT interface pins to input (SAFE-like, no contention)
	 */
	if (_verbose > 0)
		printInfo("Configuring post-programming board state...");

	ReplResult post = execRaw(R"(
from machine import Pin, PWM, mem32
import time

# RP2350 PADS_BANK0 base — used to disable pull-ups/pull-downs
# for true high-impedance on I/O pins.
PADS_BANK0 = 0x40038000

def set_hiz(gpio):
    """Set a GPIO to high-impedance input with no pulls."""
    Pin(gpio, Pin.IN)
    # Clear PUE (bit 3) and PDE (bit 2) in pad register
    addr = PADS_BANK0 + 0x04 + gpio * 4
    mem32[addr] = mem32[addr] & ~(0x0C)

# Start clock on GPIO0 (rp_projclk) at 10MHz
# The FPGA design needs a clock to operate
try:
    _clk = PWM(Pin(0))
    _clk.freq(10_000_000)
    _clk.duty_u16(32768)
except Exception:
    Pin(0, Pin.OUT, value=1)

# Release CRESET (GPIO1): drive high briefly, then switch to
# input with pull-up so the FPGA stays out of reset without
# the RP2350 actively driving the line.
Pin(1, Pin.OUT, value=1)
time.sleep_ms(1)
p1 = Pin(1, Pin.IN, Pin.PULL_UP)

# Set all other TT interface pins to high-impedance input
# (no pull-up, no pull-down) so the RP2350 doesn't interfere
# with FPGA outputs or external connections (PMOD HAT).
for gpio in (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
             17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28):
    try:
        set_hiz(gpio)
    except Exception:
        pass

print("POST_OK")
)", 5000);
	if (!post.ok || post.out.find("POST_OK") == std::string::npos) {
		printWarn("Post-programming board setup may have failed");
		if (!post.err.empty())
			printWarn("Error: " + post.err);
	}

	exitRawRepl();
	closeSerial();

	printSuccess("SRAM programming complete");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Flash Programming
 * ═══════════════════════════════════════════════════════════════════════════ */

void TTMicropython::program_flash()
{
	/* The TTDBv3 FPGA breakout board has no SPI flash chip.
	 * The iCE40UP5K has dedicated SPI pins (IOB_32a-35b) that connect
	 * via the FSPI bus to the pin header, but no flash is populated on
	 * either the FPGA breakout board or the demo PCB.
	 * Only SRAM programming is supported.
	 */
	throw std::runtime_error(
		"Flash programming is not supported on the TT FPGA Demo Board. "
		"The board has no SPI flash chip — only volatile SRAM "
		"programming (--write-sram) is available.");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Detect
 * ═══════════════════════════════════════════════════════════════════════════ */

void TTMicropython::detect()
{
	printInfo("TT FPGA: Detecting board via MicroPython raw REPL");
	printInfo("Serial port: " + _serial_port);

	openSerial(_serial_port);
	if (!enterRawRepl())
		throw std::runtime_error("Failed to enter raw REPL on " +
		                         _serial_port);

	/* Query board info */
	std::string code = R"(
import sys
print("PLATFORM:" + sys.platform)
print("VERSION:" + sys.version)
try:
    import ttboard
    print("TTBOARD:yes")
    try:
        print("TTBOARD_VER:" + ttboard.__version__)
    except AttributeError:
        print("TTBOARD_VER:unknown")
except ImportError:
    print("TTBOARD:no")
try:
    from ttboard.demoboard import DemoBoard
    tt = DemoBoard.get()
    print("BOARD_VERSION:" + str(tt.version))
except Exception:
    pass
try:
    import machine
    uid = machine.unique_id()
    uid_hex = ''.join(['%02x' % b for b in uid])
    print("UID:" + uid_hex)
except Exception:
    pass
print("DETECT_OK")
)";

	ReplResult r = execRaw(code, 5000);
	if (!r.ok || r.out.find("DETECT_OK") == std::string::npos) {
		printError("Board detection failed");
		if (!r.err.empty())
			printError("Error: " + r.err);
		exitRawRepl();
		closeSerial();
		return;
	}

	/* Parse and display results */
	std::istringstream iss(r.out);
	std::string line;
	while (std::getline(iss, line)) {
		/* Trim \r */
		if (!line.empty() && line.back() == '\r')
			line.pop_back();

		if (line.find("PLATFORM:") == 0)
			printInfo("Platform: " + line.substr(9));
		else if (line.find("VERSION:") == 0)
			printInfo("MicroPython: " + line.substr(8));
		else if (line.find("TTBOARD:yes") == 0)
			printInfo("TinyTapeout SDK: present");
		else if (line.find("TTBOARD:no") == 0)
			printInfo("TinyTapeout SDK: not found");
		else if (line.find("TTBOARD_VER:") == 0)
			printInfo("TinyTapeout SDK version: " + line.substr(12));
		else if (line.find("BOARD_VERSION:") == 0) {
			std::string ver = line.substr(14);
			printInfo("Board version: " + ver);
			if (ver.find("ttdbv3") == std::string::npos &&
			    ver.find("fpga") == std::string::npos)
				printWarn("This appears to be a TT ASIC board, "
				          "not an FPGA Demo Board. "
				          "FPGA programming is not possible.");
		} else if (line.find("UID:") == 0)
			printInfo("Board UID: " + line.substr(4));
	}

	exitRawRepl();
	closeSerial();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Auto-detection – platform-specific USB VID/PID scanning
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Target VID/PID: Raspberry Pi MicroPython
 * VID=0x2E8A, PID=0x0005 (RP2040) or PID=0x0011 (RP2350)
 */
static bool isTTMicropythonVidPid(uint16_t vid, uint16_t pid)
{
	return vid == 0x2E8A && (pid == 0x0005 || pid == 0x0011);
}

#if defined(_WIN32) || defined(_WIN64)

/* ── Windows: enumerate COM ports via SetupDi ── */
std::string TTMicropython::detectSerialPort(int8_t verbose)
{
	std::vector<std::string> candidates;

	HDEVINFO devInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT,
		NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (devInfo == INVALID_HANDLE_VALUE) {
		printError("No TinyTapeout board found (SetupDi enumeration "
		           "failed)");
		return "";
	}

	SP_DEVINFO_DATA devInfoData;
	devInfoData.cbSize = sizeof(devInfoData);

	for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devInfoData);
	     i++) {
		/* Get hardware ID to extract VID/PID */
		char hwId[256] = {0};
		if (!SetupDiGetDeviceRegistryPropertyA(devInfo, &devInfoData,
		    SPDRP_HARDWAREID, NULL, (BYTE *)hwId, sizeof(hwId),
		    NULL))
			continue;

		/* Parse VID/PID from hardware ID string like
		 * "USB\\VID_2E8A&PID_0005&..." */
		std::string hwIdStr(hwId);
		uint16_t vid = 0, pid = 0;
		size_t vpos = hwIdStr.find("VID_");
		size_t ppos = hwIdStr.find("PID_");
		if (vpos != std::string::npos)
			vid = static_cast<uint16_t>(
				strtoul(hwIdStr.c_str() + vpos + 4, NULL, 16));
		if (ppos != std::string::npos)
			pid = static_cast<uint16_t>(
				strtoul(hwIdStr.c_str() + ppos + 4, NULL, 16));

		if (!isTTMicropythonVidPid(vid, pid))
			continue;

		/* Get COM port name from registry */
		HKEY key = SetupDiOpenDevRegKey(devInfo, &devInfoData,
			DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
		if (key == INVALID_HANDLE_VALUE)
			continue;

		char portName[64] = {0};
		DWORD portNameLen = sizeof(portName);
		DWORD type = 0;
		if (RegQueryValueExA(key, "PortName", NULL, &type,
		    (BYTE *)portName, &portNameLen) == ERROR_SUCCESS) {
			if (verbose > 0)
				printInfo(std::string(portName) +
				          ": VID=" + std::to_string(vid) +
				          " PID=" + std::to_string(pid));
			candidates.push_back(std::string(portName));
		}
		RegCloseKey(key);
	}

	SetupDiDestroyDeviceInfoList(devInfo);

	if (candidates.empty()) {
		printError("No TinyTapeout board found (no RP2040/RP2350 "
		           "MicroPython COM port detected)");
		return "";
	}

	if (candidates.size() > 1) {
		printWarn("Multiple candidate devices found:");
		for (const auto &c : candidates)
			printWarn("  " + c);
		printWarn("Using first: " + candidates[0]);
		printWarn("Use -d to specify device explicitly");
	}

	if (verbose >= 0)
		printInfo("Auto-detected TT FPGA board on " + candidates[0]);

	return candidates[0];
}

#elif defined(__APPLE__)

/* ── macOS: scan /dev/cu.usbmodem* ── */
std::string TTMicropython::detectSerialPort(int8_t verbose)
{
	/* On macOS, RP2040/RP2350 MicroPython devices appear as
	 * /dev/cu.usbmodemXXXX. We can't easily read VID/PID from
	 * userspace without IOKit, so we match by device name pattern
	 * and rely on the user to specify -d if multiple devices exist.
	 */
	std::vector<std::string> candidates;

	DIR *dir = opendir("/dev");
	if (!dir) {
		printError("No TinyTapeout board found (cannot scan /dev)");
		return "";
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		std::string name(entry->d_name);
		if (name.find("cu.usbmodem") == 0) {
			std::string dev = "/dev/" + name;
			if (verbose > 0)
				printInfo("Found USB modem device: " + dev);
			candidates.push_back(dev);
		}
	}
	closedir(dir);

	if (candidates.empty()) {
		printError("No TinyTapeout board found (no /dev/cu.usbmodem* "
		           "device detected)");
		return "";
	}

	if (candidates.size() > 1) {
		printWarn("Multiple USB modem devices found:");
		for (const auto &c : candidates)
			printWarn("  " + c);
		printWarn("Using first: " + candidates[0]);
		printWarn("Use -d to specify device explicitly");
	}

	if (verbose >= 0)
		printInfo("Auto-detected TT FPGA board on " + candidates[0]);

	return candidates[0];
}

#else

/* ── Linux: scan /dev/ttyACM* + sysfs VID/PID ── */
std::string TTMicropython::detectSerialPort(int8_t verbose)
{
	std::vector<std::string> candidates;

	DIR *dir = opendir("/sys/class/tty");
	if (!dir) {
		printError("No TinyTapeout board found (cannot scan "
		           "/sys/class/tty)");
		return "";
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		std::string name(entry->d_name);
		if (name.find("ttyACM") != 0)
			continue;

		std::string dev = "/dev/" + name;
		if (access(dev.c_str(), F_OK) != 0)
			continue;

		/* Read VID/PID from sysfs */
		std::string sysfs_base = "/sys/class/tty/" + name +
		                         "/device/..";
		std::ifstream vid_file(sysfs_base + "/idVendor");
		std::ifstream pid_file(sysfs_base + "/idProduct");

		if (!vid_file.is_open() || !pid_file.is_open())
			continue;

		std::string vid_str, pid_str;
		std::getline(vid_file, vid_str);
		std::getline(pid_file, pid_str);

		uint16_t vid = static_cast<uint16_t>(
			strtoul(vid_str.c_str(), NULL, 16));
		uint16_t pid = static_cast<uint16_t>(
			strtoul(pid_str.c_str(), NULL, 16));

		if (verbose > 0)
			printInfo(name + ": VID=" + vid_str +
			          " PID=" + pid_str);

		if (isTTMicropythonVidPid(vid, pid))
			candidates.push_back(dev);
	}
	closedir(dir);

	if (candidates.empty()) {
		printError("No TinyTapeout board found (no RP2040/RP2350 "
		           "MicroPython device on /dev/ttyACM*)");
		return "";
	}

	/* Sort for deterministic selection */
	std::sort(candidates.begin(), candidates.end());

	if (candidates.size() > 1) {
		printWarn("Multiple candidate devices found:");
		for (const auto &c : candidates)
			printWarn("  " + c);
		printWarn("Using first: " + candidates[0]);
		printWarn("Use -d to specify device explicitly");
	}

	if (verbose >= 0)
		printInfo("Auto-detected TT FPGA board on " + candidates[0]);

	return candidates[0];
}

#endif /* platform detection */
