// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#ifndef SRC_ICE40GPIOSPI_HPP_
#define SRC_ICE40GPIOSPI_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "ice40SpiSlaveInterface.hpp"

/*!
 * \brief the GPIO lines (offsets on one gpiochip, or BCM GPIO numbers) of an
 *        iCE40 wired as an SPI slave to a Linux host. The host's output into
 *        the iCE40's SPI_SI is `si`; `so` is the iCE40's SPI_SO. `flash`: an
 *        SPI flash shares the bus (SPI_SS is its /CS, `si` its DO, `so` its
 *        DI); `holdn` is its /HOLD, -1 when not wired.
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
 * \file ice40GpioSpi.hpp
 * \class Ice40GpioSpi
 * \brief what every iCE40 SPI slave load over host GPIO lines shares,
 *        whatever moves the lines (libgpiod, or the Pi's GPIO registers):
 *        the line protocol, in terms of four primitives the backend gives:
 *        output(), input(), set() and get().
 *
 * Every line is an input until the Ice40 load asks for it to be driven.
 * Lines are set back to inputs in a fixed order: the bus lines (SPI_SS,
 * SPI_SO, SPI_SI, SPI_SCK, then /HOLD) first, and CRESET_B only once none
 * of them is still driven, so an iCE40 leaving reset masters its flash
 * alone.
 *
 * A flash sharing the bus is put in Deep Power-down (0xB9) when the bus is
 * taken, with the iCE40 in reset, and /HOLD is held low: the flash then
 * ignores the load. It is not woken afterwards (the loaded design owns the
 * pins then); an iCE40 booting from it sends the wake-up (0xAB) itself
 * (TN-02001 9.4). Only 0xAB, 0x05 (read status) and 0xB9 can be sent: no
 * command that writes the flash.
 */
class Ice40GpioSpi : public Ice40SpiSlaveInterface {
	public:
		void hold_reset(bool hold) override;
		bool cdone() override;
		void take_bus() override;
		void release_bus() override;
		void set_ss(bool high) override;
		void clocks(uint32_t nclk) override;

		static std::string gpio_name(unsigned int offset);

	protected:
		/*!
		 * \param[in] max_offset: the lines must be 0 .. max_offset - 1
		 * \throws when a line is out of range or given to two signals
		 */
		Ice40GpioSpi(const ice40_gpio_pins_t &pins, int8_t verbose,
			int max_offset);

		/* the backend's four primitives; each throws on failure */
		virtual void output(unsigned int offset, bool level) = 0;
		virtual void input(unsigned int offset) = 0;
		virtual void set(unsigned int offset, bool level) = 0;
		virtual bool get(unsigned int offset) = 0;

		/* each of offsets an input, all tried: what failed, in words */
		std::vector<std::string> release(
			const std::vector<unsigned int> &offsets);
		/* the lines driven now but CRESET_B, then CRESET_B if none of
		 * those is stuck: what failed, in words */
		std::vector<std::string> release_all();

		ice40_gpio_pins_t _pins;
		bool _verbose;
		/* request order, also the v1 release order: CRESET_B last */
		std::vector<unsigned int> _lines;
		/* bus lines take_bus() drives, in release order */
		std::vector<unsigned int> _bus;
		/* the lines driven now, and their level */
		std::map<unsigned int, bool> _outputs;

	private:
		uint8_t flash_command(uint8_t opcode, bool read);
		void flash_power_down();
};

#endif  // SRC_ICE40GPIOSPI_HPP_
