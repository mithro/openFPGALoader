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

#include "ice40GpioSpi.hpp"

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
class LibgpiodIce40Spi : public Ice40GpioSpi {
	public:
		/*!
		 * \param[in] dev: /dev/gpiochipN; empty: the Raspberry Pi header's
		 *            chip, found by its label (header_chip())
		 */
		LibgpiodIce40Spi(const ice40_gpio_pins_t &pins,
			const std::string &dev, int8_t verbose);
		~LibgpiodIce40Spi();

		void release_bus() override;
		void write(const uint8_t *data, uint32_t len) override;

		/*!
		 * \brief the gpiochip of a Raspberry Pi's 40-pin header: the one
		 *        chip whose label is pinctrl-rp1 (Pi 5), pinctrl-bcm2711
		 *        (Pi 4) or pinctrl-bcm2835 (Pi 1-3); throws unless exactly
		 *        one is found
		 */
		static std::string header_chip();

	private:
		void output(unsigned int offset, bool level) override;
		void input(unsigned int offset) override;
		void set(unsigned int offset, bool level) override;
		bool get(unsigned int offset) override;
		void close_lines();
#ifdef GPIOD_APIV2
		void reconfigure();
#endif

		gpiod_chip *_chip;
#ifdef GPIOD_APIV2
		gpiod_line_request *_request;
		unsigned int _bit_pins[2];  /* SPI_SCK, SPI_SI */
#else
		std::map<unsigned int, gpiod_line *> _line;
#endif
};

#endif  // SRC_LIBGPIODICE40SPI_HPP_
