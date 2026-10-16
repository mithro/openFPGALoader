// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#include "ice40GpioSpi.hpp"

#include <unistd.h>

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

std::string Ice40GpioSpi::gpio_name(unsigned int offset)
{
	return "GPIO" + std::to_string(offset);
}

Ice40GpioSpi::Ice40GpioSpi(const ice40_gpio_pins_t &pins, int8_t verbose,
		int max_offset):
	_pins(pins), _verbose(verbose > 0)
{
	/* the order fomu_header_id.py (fpgas.online-test-designs) requests
	 * and, on libgpiod v1, gives the lines back in: CRESET_B last
	 */
	const struct { const char *name; int offset; } lines[] = {
		{"CDONE", _pins.cdone}, {"SPI_SS", _pins.ss}, {"SPI_SO", _pins.so},
		{"SPI_SI", _pins.si}, {"SPI_SCK", _pins.sck},
		{"/HOLD", _pins.holdn}, {"CRESET_B", _pins.creset}};
	for (const auto &l : lines) {
		if (l.offset == -1 && std::string(l.name) == "/HOLD")
			continue;  // no flash /HOLD on this board
		if (l.offset < 0 || l.offset >= max_offset)
			throw std::runtime_error(std::string(l.name) + ": GPIO " +
				std::to_string(l.offset) + " is outside of the valid range");
		if (std::find(_lines.begin(), _lines.end(),
				static_cast<unsigned int>(l.offset)) != _lines.end())
			throw std::runtime_error(std::string(l.name) + ": " +
				gpio_name(l.offset) + " is given to two signals");
		_lines.push_back(l.offset);
	}
	_bus.push_back(_pins.ss);
	_bus.push_back(_pins.so);
	_bus.push_back(_pins.si);
	_bus.push_back(_pins.sck);
	if (_pins.holdn != -1)
		_bus.push_back(_pins.holdn);
}

std::vector<std::string> Ice40GpioSpi::release(
		const std::vector<unsigned int> &offsets)
{
	std::vector<std::string> stuck;
	for (unsigned int offset : offsets) {
		try {
			input(offset);
		} catch (std::exception &e) {
			stuck.push_back(e.what());
		}
	}
	return stuck;
}

/* every bus line an input, then CRESET_B, but only if none of the others
 * is stuck
 */
std::vector<std::string> Ice40GpioSpi::release_all()
{
	std::vector<unsigned int> driven;
	for (auto &o : _outputs)
		if (o.first != static_cast<unsigned int>(_pins.creset))
			driven.push_back(o.first);
	std::vector<std::string> stuck = release(driven);
	if (_outputs.count(_pins.creset)) {
		if (stuck.empty()) {
			std::vector<std::string> e = release({
				static_cast<unsigned int>(_pins.creset)});
			stuck.insert(stuck.end(), e.begin(), e.end());
		} else {
			stuck.push_back("CRESET_B was not let go before the lines "
				"were given back");
		}
	}
	return stuck;
}

void Ice40GpioSpi::hold_reset(bool hold)
{
	if (hold)
		output(_pins.creset, false);
	else
		input(_pins.creset);
}

bool Ice40GpioSpi::cdone()
{
	return get(_pins.cdone);
}

/* flash opcodes this class can send: none of them writes the flash */
#define FLASH_RELEASE_POWER_DOWN	0xAB
#define FLASH_READ_STATUS_1			0x05
#define FLASH_POWER_DOWN			0xB9
#define FLASH_BUSY					0x01	/* status register 1 */
#define FLASH_T_US					1000	/* tRES1, tDP: 3 us (W25Q128JV) */
#define FLASH_BUSY_WAIT_MS			5000

/* one flash command, SPI mode 0, msb first, on SPI_SO (the flash's DI),
 * with the iCE40 in reset; with read, one byte back from SPI_SI (its DO)
 */
uint8_t Ice40GpioSpi::flash_command(uint8_t opcode, bool read)
{
	if (opcode != FLASH_RELEASE_POWER_DOWN &&
			opcode != FLASH_READ_STATUS_1 && opcode != FLASH_POWER_DOWN)
		throw std::logic_error("flash opcode not allowed: not sent");
	uint8_t rx = 0;
	set(_pins.ss, false);
	for (int b = 7; b >= 0; b--) {
		abort_check();
		set(_pins.so, (opcode >> b) & 1);
		set(_pins.sck, true);
		set(_pins.sck, false);
	}
	set(_pins.so, false);
	if (read) {
		for (int b = 7; b >= 0; b--) {
			abort_check();
			set(_pins.sck, true);
			rx = (rx << 1) | (get(_pins.si) ? 1 : 0);
			set(_pins.sck, false);
		}
	}
	set(_pins.ss, true);
	return rx;
}

/* the flash in Deep Power-down, so that it ignores the load whatever its
 * QE bit (with QE=1, /HOLD is a data pin and holding it low does not
 * pause the flash): woken first, as a previous load leaves it asleep and
 * a sleeping flash reads no status; an erase or program a design left
 * going must end first (a busy flash ignores 0xB9)
 */
void Ice40GpioSpi::flash_power_down()
{
	output(_pins.so, false);
	if (_pins.holdn != -1)
		output(_pins.holdn, true);
	flash_command(FLASH_RELEASE_POWER_DOWN, false);
	usleep(FLASH_T_US);
	uint8_t status = flash_command(FLASH_READ_STATUS_1, true);
	for (int ms = 0; (status & FLASH_BUSY) && ms < FLASH_BUSY_WAIT_MS;
			ms += 10) {
		abort_check();
		usleep(10000);
		status = flash_command(FLASH_READ_STATUS_1, true);
	}
	if (status & FLASH_BUSY)
		throw std::runtime_error("the flash stayed busy (status register 1 "
			"bit 0) for " + std::to_string(FLASH_BUSY_WAIT_MS / 1000) +
			" s: not put in power-down, nothing loaded");
	flash_command(FLASH_POWER_DOWN, false);
	usleep(FLASH_T_US);
	input(_pins.so);
}

/* with the iCE40 in reset (its SPI pins released): SPI_SS high first, so
 * nothing on the bus is selected while the other lines move, and SPI_SCK
 * low (mode 0); a flash on the bus is put in Deep Power-down and /HOLD
 * held low ("To program FPGA, keep SPI_IO3 LOW", Fomu EVT schematic; it
 * pauses a flash whose QE bit is 0); SPI_SO is an input again; SPI_SI high
 */
void Ice40GpioSpi::take_bus()
{
	output(_pins.ss, true);
	output(_pins.sck, false);
	if (_pins.flash)
		flash_power_down();
	if (_pins.holdn != -1)
		output(_pins.holdn, false);
	output(_pins.si, true);
}

/* every bus line an input again, one by one in _bus order, all tried */
void Ice40GpioSpi::release_bus()
{
	std::string why;
	for (const std::string &s : release(_bus))
		why += (why.empty() ? "" : "; ") + s;
	if (!why.empty())
		throw std::runtime_error(why);
}

void Ice40GpioSpi::set_ss(bool high)
{
	set(_pins.ss, high);
}

void Ice40GpioSpi::clocks(uint32_t nclk)
{
	set(_pins.si, true);
	for (uint32_t i = 0; i < nclk; i++) {
		abort_check();
		set(_pins.sck, true);
		set(_pins.sck, false);
	}
}
