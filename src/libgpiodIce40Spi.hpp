// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#ifndef SRC_LIBGPIODICE40SPI_HPP_
#define SRC_LIBGPIODICE40SPI_HPP_

#include <map>
#include <string>
#include <vector>

#include <gpiod.h>

#include "ice40SpiSlaveInterface.hpp"

/*!
 * \brief the GPIO lines (offsets on one gpiochip) of an iCE40 wired as an
 *        SPI slave to a Linux host. The host's output into the iCE40's
 *        SPI_SI is `si`; `so` is the iCE40's SPI_SO. `flash`: an SPI
 *        flash shares the bus (SPI_SS is its /CS, `si` its DO, `so` its DI);
 *        `holdn` is its /HOLD, -1 when not wired.
 */
typedef struct {
	int si;
	int so;
	int sck;
	int ss;
	int holdn;
	int creset;
	int cdone;
	bool flash;
} ice40_gpio_pins_t;

/*!
 * \file libgpiodIce40Spi.hpp
 * \class LibgpiodIce40Spi
 * \brief iCE40 SPI slave configuration lines bit-banged through libgpiod
 *        (v1 or v2 API, as LibgpiodJtagBitbang)
 *
 * Every line is requested as an input and is only driven when the Ice40
 * load asks for it. Lines are set back to inputs in a fixed order: the bus
 * lines (SPI_SS, SPI_SO, SPI_SI, SPI_SCK, then /HOLD) first, and CRESET_B
 * only once none of them is still driven, so an iCE40 leaving reset
 * masters its flash alone. Every libgpiod call is checked; a failure
 * throws.
 *
 * A flash sharing the bus is put in Deep Power-down (0xB9) when the bus is
 * taken, with the iCE40 in reset, and /HOLD is held low: the flash then
 * ignores the load. It is not woken afterwards (the loaded design owns the
 * pins then); an iCE40 booting from it sends the wake-up (0xAB) itself
 * (TN-02001 9.4). Only 0xAB, 0x05 (read status) and 0xB9 can be sent: no
 * command that writes the flash.
 */
class LibgpiodIce40Spi : public Ice40SpiSlaveInterface {
	public:
		/*!
		 * \param[in] dev: /dev/gpiochipN; empty: the Raspberry Pi header's
		 *            chip, found by its label (header_chip())
		 */
		LibgpiodIce40Spi(const ice40_gpio_pins_t &pins,
			const std::string &dev, int8_t verbose);
		~LibgpiodIce40Spi();

		void hold_reset(bool hold) override;
		bool cdone() override;
		void take_bus() override;
		void release_bus() override;
		void set_ss(bool high) override;
		void write(const uint8_t *data, uint32_t len) override;
		void clocks(uint32_t nclk) override;

		/*!
		 * \brief the gpiochip of a Raspberry Pi's 40-pin header: the one
		 *        chip whose label is pinctrl-rp1 (Pi 5), pinctrl-bcm2711
		 *        (Pi 4) or pinctrl-bcm2835 (Pi 1-3); throws unless exactly
		 *        one is found
		 */
		static std::string header_chip();

	private:
		void output(unsigned int offset, bool level);
		void input(unsigned int offset);
		void set(unsigned int offset, bool level);
		bool get(unsigned int offset);
		/* each of offsets an input, all tried: what failed, in words */
		std::vector<std::string> release(
			const std::vector<unsigned int> &offsets);
		void close_lines();
		uint8_t flash_command(uint8_t opcode, bool read);
		void flash_power_down();
#ifdef GPIOD_APIV2
		void reconfigure();
#endif

		ice40_gpio_pins_t _pins;
		bool _verbose;
		/* request order, also the v1 release order: CRESET_B last */
		std::vector<unsigned int> _lines;
		/* bus lines take_bus() drives, in release order */
		std::vector<unsigned int> _bus;
		/* the lines driven now, and their level */
		std::map<unsigned int, bool> _outputs;

		gpiod_chip *_chip;
#ifdef GPIOD_APIV2
		gpiod_line_request *_request;
		unsigned int _bit_pins[2];  /* SPI_SCK, SPI_SI */
#else
		std::map<unsigned int, gpiod_line *> _line;
#endif
};

#endif  // SRC_LIBGPIODICE40SPI_HPP_
