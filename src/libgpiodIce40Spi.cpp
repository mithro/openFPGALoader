// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#include "libgpiodIce40Spi.hpp"

#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "display.hpp"

static std::string gpio_name(unsigned int offset)
{
	return Ice40GpioSpi::gpio_name(offset);
}

static std::runtime_error gpio_error(const std::string &what,
		unsigned int offset)
{
	return std::runtime_error(what + " " + gpio_name(offset) + ": " +
		strerror(errno));
}

std::string LibgpiodIce40Spi::header_chip()
{
	static const std::set<std::string> labels = {
		"pinctrl-rp1", "pinctrl-bcm2711", "pinctrl-bcm2835"};

	DIR *dir = opendir("/dev");
	if (!dir)
		throw std::runtime_error(std::string("cannot list /dev: ") +
			strerror(errno));
	std::vector<std::string> chips;
	struct dirent *ent;
	while ((ent = readdir(dir)) != NULL) {
		std::string name = ent->d_name;
		if (name.compare(0, 8, "gpiochip") == 0)
			chips.push_back("/dev/" + name);
	}
	closedir(dir);
	std::sort(chips.begin(), chips.end());

	std::vector<std::string> found, unreadable;
	std::set<dev_t> seen;  // a /dev/gpiochipN symlink to another chip
	for (const std::string &path : chips) {
		struct stat st;
		if (stat(path.c_str(), &st) != 0) {
			unreadable.push_back(path + ": " + strerror(errno));
			continue;
		}
		if (!seen.insert(st.st_rdev).second)
			continue;
		gpiod_chip *chip = gpiod_chip_open(path.c_str());
		if (!chip) {
			unreadable.push_back(path + ": " + strerror(errno));
			continue;
		}
#ifdef GPIOD_APIV2
		gpiod_chip_info *info = gpiod_chip_get_info(chip);
		if (!info) {
			unreadable.push_back(path + ": " + strerror(errno));
			gpiod_chip_close(chip);
			continue;
		}
		std::string label = gpiod_chip_info_get_label(info);
		gpiod_chip_info_free(info);
#else
		std::string label = gpiod_chip_label(chip);
#endif
		gpiod_chip_close(chip);
		if (labels.count(label))
			found.push_back(path);
	}

	if (found.size() == 1)
		return found[0];

	std::string why = found.empty() ?
		"no GPIO chip labelled pinctrl-rp1, pinctrl-bcm2711 or "
		"pinctrl-bcm2835 (a Raspberry Pi's header)" :
		"more than one GPIO chip with a Raspberry Pi header's label";
	for (const std::string &path : found)
		why += "; " + path;
	for (const std::string &u : unreadable)
		why += "; cannot open " + u;
	throw std::runtime_error(why + ": give the chip with -d /dev/gpiochipN");
}

LibgpiodIce40Spi::LibgpiodIce40Spi(const ice40_gpio_pins_t &pins,
		const std::string &dev, int8_t verbose):
	Ice40GpioSpi(pins, verbose, 1000), _chip(NULL)
#ifdef GPIOD_APIV2
	, _request(NULL)
#endif
{
	std::string chip_dev = dev.empty() ? header_chip() : dev;
	if (chip_dev.compare(0, 13, "/dev/gpiochip") != 0 ||
			chip_dev.length() < 14)
		throw std::runtime_error("invalid gpio chip " + chip_dev +
			": should be /dev/gpiochipN");

	if (_verbose)
		printInfo("libgpiod iCE40 SPI: " + chip_dev + ", SI=" +
			gpio_name(_pins.si) + " SO=" + gpio_name(_pins.so) + " SCK=" +
			gpio_name(_pins.sck) + " SS=" + gpio_name(_pins.ss) + " /HOLD=" +
			(_pins.holdn == -1 ? std::string("none") : gpio_name(_pins.holdn)) +
			" CRESET_B=" + gpio_name(_pins.creset) + " CDONE=" +
			gpio_name(_pins.cdone) + (_pins.flash ? ", flash on the bus" :
			", no flash on the bus"));

	_chip = gpiod_chip_open(chip_dev.c_str());
	if (!_chip)
		throw std::runtime_error("unable to open " + chip_dev + ": " +
			strerror(errno));

#ifdef GPIOD_APIV2
	_bit_pins[0] = _pins.sck;
	_bit_pins[1] = _pins.si;
	gpiod_request_config *req_cfg = gpiod_request_config_new();
	if (!req_cfg) {
		gpiod_chip_close(_chip);
		throw std::runtime_error("libgpiod: out of memory");
	}
	gpiod_request_config_set_consumer(req_cfg, "openFPGALoader_ice40");
	gpiod_line_settings *in = gpiod_line_settings_new();
	gpiod_line_config *cfg = gpiod_line_config_new();
	int ret = -1;
	if (in && cfg &&
			gpiod_line_settings_set_direction(in,
				GPIOD_LINE_DIRECTION_INPUT) == 0)
		ret = gpiod_line_config_add_line_settings(cfg, _lines.data(),
			_lines.size(), in);
	if (ret == 0)
		_request = gpiod_chip_request_lines(_chip, req_cfg, cfg);
	int err = errno;
	gpiod_line_config_free(cfg);
	gpiod_line_settings_free(in);
	gpiod_request_config_free(req_cfg);
	if (!_request) {
		gpiod_chip_close(_chip);
		throw std::runtime_error("unable to request the GPIO lines as "
			"inputs on " + chip_dev + ": " + strerror(err));
	}
#else
	for (unsigned int offset : _lines) {
		gpiod_line *line = gpiod_chip_get_line(_chip, offset);
		if (!line || gpiod_line_request_input(line,
				"openFPGALoader_ice40") < 0) {
			int err = errno;
			for (auto &l : _line)
				gpiod_line_release(l.second);
			gpiod_chip_close(_chip);
			throw std::runtime_error("unable to request " +
				gpio_name(offset) + " as an input on " + chip_dev + ": " +
				strerror(err));
		}
		_line[offset] = line;
	}
#endif
}

LibgpiodIce40Spi::~LibgpiodIce40Spi()
{
	close_lines();
}

/* every bus line an input, then CRESET_B, but only if none of the others
 * is stuck; then the lines are given back to the kernel (one release on
 * v2, line by line in request order, CRESET_B last, on v1)
 */
void LibgpiodIce40Spi::close_lines()
{
	std::vector<std::string> stuck = release_all();
#ifdef GPIOD_APIV2
	if (_request)
		gpiod_line_request_release(_request);
#else
	for (unsigned int offset : _lines)
		gpiod_line_release(_line[offset]);
#endif
	if (_chip)
		gpiod_chip_close(_chip);
	for (const std::string &s : stuck)
		printError("libgpiod iCE40 SPI: " + s);
}

#ifdef GPIOD_APIV2
/* v2 changes direction by reconfiguring the whole request: every line,
 * the inputs and the outputs (with their levels in _outputs)
 */
void LibgpiodIce40Spi::reconfigure()
{
	gpiod_line_config *cfg = gpiod_line_config_new();
	if (!cfg)
		throw std::runtime_error("libgpiod: out of memory");
	int ret = 0;
	for (unsigned int offset : _lines) {
		gpiod_line_settings *s = gpiod_line_settings_new();
		if (!s) {
			ret = -1;
			break;
		}
		auto o = _outputs.find(offset);
		if (o == _outputs.end()) {
			ret = gpiod_line_settings_set_direction(s,
				GPIOD_LINE_DIRECTION_INPUT);
		} else {
			ret = gpiod_line_settings_set_direction(s,
				GPIOD_LINE_DIRECTION_OUTPUT);
			if (ret == 0)
				ret = gpiod_line_settings_set_output_value(s, o->second ?
					GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
		}
		if (ret == 0)
			ret = gpiod_line_config_add_line_settings(cfg, &offset, 1, s);
		gpiod_line_settings_free(s);
		if (ret != 0)
			break;
	}
	if (ret == 0)
		ret = gpiod_line_request_reconfigure_lines(_request, cfg);
	int err = errno;
	gpiod_line_config_free(cfg);
	if (ret != 0)
		throw std::runtime_error(std::string("unable to reconfigure the "
			"GPIO lines: ") + strerror(err));
}
#endif

void LibgpiodIce40Spi::output(unsigned int offset, bool level)
{
#ifdef GPIOD_APIV2
	auto prev = _outputs.find(offset);
	bool was_output = prev != _outputs.end();
	bool prev_level = was_output && prev->second;
	_outputs[offset] = level;
	try {
		reconfigure();
	} catch (std::exception &e) {
		if (was_output)
			_outputs[offset] = prev_level;
		else
			_outputs.erase(offset);
		throw std::runtime_error("unable to drive " + gpio_name(offset) +
			": " + e.what());
	}
#else
	if (gpiod_line_set_direction_output(_line[offset], level) < 0)
		throw gpio_error("unable to drive", offset);
	_outputs[offset] = level;
#endif
}

/* offset an input; it stays in _outputs, as driven, unless that worked */
void LibgpiodIce40Spi::input(unsigned int offset)
{
#ifdef GPIOD_APIV2
	auto prev = _outputs.find(offset);
	if (prev == _outputs.end())
		return;
	bool prev_level = prev->second;
	_outputs.erase(prev);
	try {
		reconfigure();
	} catch (std::exception &e) {
		_outputs[offset] = prev_level;
		throw std::runtime_error("unable to set " + gpio_name(offset) +
			" back to an input: " + e.what());
	}
#else
	if (gpiod_line_set_direction_input(_line[offset]) < 0)
		throw gpio_error("unable to set back to an input", offset);
	_outputs.erase(offset);
#endif
}

void LibgpiodIce40Spi::set(unsigned int offset, bool level)
{
	if (!_outputs.count(offset))
		throw std::runtime_error(gpio_name(offset) + " is not driven");
#ifdef GPIOD_APIV2
	if (gpiod_line_request_set_value(_request, offset, level ?
			GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) < 0)
#else
	if (gpiod_line_set_value(_line[offset], level) < 0)
#endif
		throw gpio_error("unable to set", offset);
	_outputs[offset] = level;
}

bool LibgpiodIce40Spi::get(unsigned int offset)
{
#ifdef GPIOD_APIV2
	gpiod_line_value v = gpiod_line_request_get_value(_request, offset);
	if (v == GPIOD_LINE_VALUE_ERROR)
		throw gpio_error("unable to read", offset);
	return v == GPIOD_LINE_VALUE_ACTIVE;
#else
	int v = gpiod_line_get_value(_line[offset]);
	if (v < 0)
		throw gpio_error("unable to read", offset);
	return v == 1;
#endif
}

/* v2: every bus line an input in one reconfigure call, so that the Pi
 * stops driving them all at once (after a load the loaded design owns
 * these pins). v1 requests each line on its own, so each takes its own
 * call, in _bus order. If the one v2 call fails, the lines are tried one
 * by one to find which is stuck, and the failure is reported either way.
 */
void LibgpiodIce40Spi::release_bus()
{
	std::string why;
#ifdef GPIOD_APIV2
	std::map<unsigned int, bool> saved = _outputs;
	for (unsigned int offset : _bus)
		_outputs.erase(offset);
	try {
		reconfigure();
		return;
	} catch (std::exception &e) {
		_outputs = saved;
		why = std::string("releasing the bus lines in one call: ") + e.what();
	}
#endif
	std::vector<std::string> stuck = release(_bus);
	for (const std::string &s : stuck)
		why += (why.empty() ? "" : "; ") + s;
	if (!why.empty())
		throw std::runtime_error(why);
}

/* mode 0, msb first: SPI_SI changes as SPI_SCK falls and is captured as it
 * rises. v2: one call sets SPI_SCK low and SPI_SI together, one call sets
 * SPI_SCK high. v1: SPI_SI only when it changes, then SPI_SCK high, low.
 */
void LibgpiodIce40Spi::write(const uint8_t *data, uint32_t len)
{
	if (!_outputs.count(_pins.sck) || !_outputs.count(_pins.si))
		throw std::runtime_error("SPI_SCK and SPI_SI are not driven");
	bool si = _outputs[_pins.si];
#ifdef GPIOD_APIV2
	gpiod_line_value vals[2];
	vals[0] = GPIOD_LINE_VALUE_INACTIVE;
	for (uint32_t i = 0; i < len; i++) {
		for (int b = 7; b >= 0; b--) {
			if (abort_pending()) {
				/* SPI_SCK is left high between bits here */
				_outputs[_pins.sck] = (i != 0 || b != 7);
				_outputs[_pins.si] = si;
				abort_check();
			}
			si = (data[i] >> b) & 1;
			vals[1] = si ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
			if (gpiod_line_request_set_values_subset(_request, 2,
					_bit_pins, vals) < 0)
				throw gpio_error("unable to set SPI_SCK/SPI_SI", _pins.sck);
			if (gpiod_line_request_set_value(_request, _pins.sck,
					GPIOD_LINE_VALUE_ACTIVE) < 0)
				throw gpio_error("unable to set", _pins.sck);
		}
	}
	if (gpiod_line_request_set_value(_request, _pins.sck,
			GPIOD_LINE_VALUE_INACTIVE) < 0)
		throw gpio_error("unable to set", _pins.sck);
#else
	gpiod_line *sck = _line[_pins.sck];
	gpiod_line *si_line = _line[_pins.si];
	for (uint32_t i = 0; i < len; i++) {
		for (int b = 7; b >= 0; b--) {
			if (abort_pending()) {
				_outputs[_pins.sck] = false;
				_outputs[_pins.si] = si;
				abort_check();
			}
			bool bit = (data[i] >> b) & 1;
			if (bit != si) {
				if (gpiod_line_set_value(si_line, bit) < 0)
					throw gpio_error("unable to set", _pins.si);
				si = bit;
			}
			if (gpiod_line_set_value(sck, 1) < 0 ||
					gpiod_line_set_value(sck, 0) < 0)
				throw gpio_error("unable to set", _pins.sck);
		}
	}
#endif
	_outputs[_pins.sck] = false;
	_outputs[_pins.si] = si;
}
