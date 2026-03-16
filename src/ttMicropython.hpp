// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2025 Tim Ansell <me@mith.ro>
 *
 * TinyTapeout FPGA Demo Board support via MicroPython raw REPL.
 * Programs iCE40UP5K FPGA through RP2040/RP2350 running MicroPython.
 */

#ifndef SRC_TTMICROPYTHON_HPP_
#define SRC_TTMICROPYTHON_HPP_

#include <cstdint>
#include <string>

class TTMicropython {
 public:
	TTMicropython(const std::string &filename, const std::string &file_type,
	              const std::string &serial_port, bool write_flash,
	              bool verify, int8_t verbose);
	~TTMicropython();

	/* Auto-detect a TT FPGA Demo Board serial port.
	 * Scans /dev/ttyACM* and checks USB VID/PID via sysfs.
	 */
	static std::string detectSerialPort(int8_t verbose);

	void program_sram();     /* iCE40 CRAM load via SPI */
	void program_flash();    /* Not supported — no flash on TTDBv3 */
	void detect();           /* Probe board, print info */

 private:
	/* Serial port (POSIX termios) */
	int openSerial(const std::string &port);
	void closeSerial();
	ssize_t serialWrite(const uint8_t *data, size_t len);
	ssize_t serialRead(uint8_t *buf, size_t maxlen, int timeout_ms);
	void drainSerial();

	/* MicroPython raw REPL protocol */
	bool enterRawRepl();
	void exitRawRepl();
	struct ReplResult {
		std::string out;
		std::string err;
		bool ok;
	};
	ReplResult execRaw(const std::string &code, int timeout_ms = 10000);

	/* iCE40 SRAM programming via SPI */
	bool setupSramProgramming();
	bool sendBitstreamChunk(const uint8_t *data, size_t len);
	bool finalizeSramProgramming();

	int _fd;
	std::string _filename;
	std::string _file_type;
	std::string _serial_port;
	bool _write_flash;
	bool _verify;
	int8_t _verbose;
	bool _in_raw_repl;
};

#endif  // SRC_TTMICROPYTHON_HPP_
