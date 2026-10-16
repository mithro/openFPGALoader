// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#ifndef SRC_GPIOMEMICE40SPI_HPP_
#define SRC_GPIOMEMICE40SPI_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "ice40GpioSpi.hpp"

/*!
 * \file gpiomemIce40Spi.hpp
 * \class Ice40GpioRegs
 * \brief the Raspberry Pi GPIO register page, as the iCE40 load sees it:
 *        32-bit reads and writes at a byte offset into the GPIO block.
 *        Real hardware is GpiomemRegs; a test gives a model of the
 *        registers (and of what is wired to them).
 */
class Ice40GpioRegs {
	public:
		virtual ~Ice40GpioRegs() {}
		virtual uint32_t read(uint32_t offset) = 0;
		virtual void write(uint32_t offset, uint32_t value) = 0;
};

/*!
 * \class GpiomemRegs
 * \brief /dev/gpiomem mapped read/write: one 4 KiB page at file offset 0,
 *        the GPIO register block of the BCM2835, BCM2836, BCM2837 and
 *        BCM2711 (Raspberry Pi 1 to 4). Every access is a volatile 32-bit
 *        load or store.
 */
class GpiomemRegs : public Ice40GpioRegs {
	public:
		/*!
		 * \param[in] path: the device (a test gives an ordinary file)
		 * \throws when it cannot be opened read/write or mapped, with why
		 *         and what to check
		 */
		explicit GpiomemRegs(const std::string &path = "/dev/gpiomem");
		~GpiomemRegs();
		GpiomemRegs(const GpiomemRegs &) = delete;
		GpiomemRegs &operator=(const GpiomemRegs &) = delete;

		uint32_t read(uint32_t offset) override {
			return *reinterpret_cast<volatile uint32_t *>(
				reinterpret_cast<volatile uint8_t *>(_page) + offset);
		}
		void write(uint32_t offset, uint32_t value) override {
			*reinterpret_cast<volatile uint32_t *>(
				reinterpret_cast<volatile uint8_t *>(_page) + offset) = value;
		}

	private:
		void *_page;
};

/*!
 * \class GpiomemIce40Spi
 * \brief iCE40 SPI slave configuration lines bit-banged straight through
 *        the GPIO registers of a BCM283x / BCM2711 Raspberry Pi (1 to 4):
 *        a load libgpiod cannot keep inside the iCE40's 1 MHz minimum
 *        slave configuration clock (DS-02008 Table 4.24).
 *
 * Registers (offsets from the GPIO block, 32-bit, BCM2835 ARM Peripherals
 * section 6.1, BCM2711 ARM Peripherals section 5.2): GPFSELn 0x00-0x14
 * (3 bits per pin, 000 input, 001 output), GPSET0 0x1C, GPCLR0 0x28,
 * GPLEV0 0x34. Only GPIO 0-31 (the "0" registers); the pull registers are
 * never touched.
 *
 * The kernel does not know of these accesses: nothing stops another driver
 * or process using the same pins. A pin whose function is an alternate one
 * (a driver's, SPI0 for instance) is refused rather than taken over.
 *
 * Each pin's function (its GPFSEL bits) is saved before the pin is first
 * changed and put back by restore_functions(), which the destructor runs
 * on every way out of a load, errors and the SIGINT/SIGTERM/SIGHUP stop
 * included (they unwind to the destructor, see Ice40::program_cram_slave).
 *
 * The image is clocked by write() and clocks(), whose loop is timed and
 * padded, before every call, to a period not under 1/20 MHz: the
 * data sheet's 25 MHz maximum is never approached, whatever the CPU runs
 * at; Ice40's per-chunk and overall rate checks remain the authority.
 */
class GpiomemIce40Spi : public Ice40GpioSpi {
	public:
		/*!
		 * \param[in] regs: the GPIO registers (takes them over)
		 * \throws when a line is not GPIO 0-31, or its function is an
		 *         alternate one
		 */
		GpiomemIce40Spi(const ice40_gpio_pins_t &pins,
			std::unique_ptr<Ice40GpioRegs> regs, int8_t verbose);
		~GpiomemIce40Spi();

		void write(const uint8_t *data, uint32_t len) override;
		void clocks(uint32_t nclk) override;

		/*!
		 * \brief every pin changed so far back to the function it had
		 *        when the load started, and read back. Plain register
		 *        accesses only: no allocation, no exception, no libc call,
		 *        so it is async-signal-safe.
		 * \return how many pins did not read back as saved
		 */
		int restore_functions() noexcept;

		/*! \brief the spins of padding after each SPI_SCK edge now in use */
		uint32_t pad_spins() const { return _pad_spins; }
		/*!
		 * \brief the padding for a host whose three register writes of a
		 *        bit take bit_ns together and whose spin takes spin_ns:
		 *        SPI_SCK high w + pad >= 25 ns, low 2w + pad >= 25 ns,
		 *        period 3w + 2 pad >= 50 ns (w = bit_ns / 3)
		 */
		static uint32_t pad_spins_for(double bit_ns, double spin_ns);

		/*! which backend runs an iCE40 SRAM load, and why (for the log) */
		struct Choice {
			bool gpiomem;
			std::string why;
		};
		/*!
		 * \brief gpiomem if the host is a BCM2835, BCM2836, BCM2837 or
		 *        BCM2711 (named in `compatible`, the device tree's
		 *        NUL-separated list), else libgpiod; libgpiod too when the
		 *        user named a gpiochip.
		 * \param[in] chip_given: -d was given
		 * \param[in] compatible: /proc/device-tree/compatible
		 */
		static Choice select(bool chip_given,
			const std::string &compatible = "/proc/device-tree/compatible");

		/*!
		 * \brief a GpiomemRegs on /dev/gpiomem for a BCM283x host, or the
		 *        loud failure: why it cannot be had, "is the user in the
		 *        gpio group?", and that libgpiod is not used instead
		 */
		static std::unique_ptr<Ice40GpioRegs> open_gpiomem(
			const std::string &path = "/dev/gpiomem");

	private:
		void output(unsigned int offset, bool level) override;
		void input(unsigned int offset) override;
		void set(unsigned int offset, bool level) override;
		bool get(unsigned int offset) override;

		void fsel(unsigned int offset, uint32_t function);
		void mark_changed(unsigned int offset);
		void bits(const uint8_t *data, uint32_t len);
		void calibrate();

		std::unique_ptr<Ice40GpioRegs> _regs;
		/* each pin's function when the load started: fixed storage, so
		 * restore_functions() allocates nothing (at most 7 lines)
		 */
		struct Saved {
			uint32_t pin;
			uint32_t function;
			bool changed;
		};
		Saved _saved[8];
		unsigned int _n_saved;
		uint32_t _pad_spins;
};

#endif  // SRC_GPIOMEMICE40SPI_HPP_
