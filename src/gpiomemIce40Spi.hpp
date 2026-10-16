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
 * \brief the CPU clock, in kHz, as cpufreq reports it
 */
struct CpuFreq {
	double cur_khz;
	double min_khz;
	double max_khz;
};

/*!
 * \file gpiomemIce40Spi.hpp
 * \class Ice40GpioRegs
 * \brief what the iCE40 load needs from a Raspberry Pi: the GPIO register
 *        page (32-bit reads and writes at a byte offset into the GPIO
 *        block), a clock, a way to burn time, and the CPU frequency. The
 *        defaults are the real ones (steady_clock, a counted loop, cpufreq
 *        in sysfs); real registers are GpiomemRegs. A test overrides all of
 *        them to keep a virtual clock: every register access and every
 *        spin advances it, and the SPI_SCK edges can then be timed exactly.
 */
class Ice40GpioRegs {
	public:
		virtual ~Ice40GpioRegs() {}
		virtual uint32_t read(uint32_t offset) = 0;
		virtual void write(uint32_t offset, uint32_t value) = 0;

		/*! \brief a monotonic time in ns */
		virtual double now_ns();
		/*! \brief burn n loop iterations (each a CPU-bound spin) */
		virtual void spin(uint32_t n);
		/*! \brief the CPU clock now, and its limits. Throws, saying which
		 *         file, when cpufreq cannot be read: nothing is guessed. */
		virtual CpuFreq cpu_freq();

		/*!
		 * \brief cpufreq in `dir` (default: cpu0's policy): scaling_cur_freq,
		 *        cpuinfo_min_freq, cpuinfo_max_freq, in kHz; throws if one
		 *        cannot be read or is not a positive number
		 */
		static CpuFreq read_cpufreq(const std::string &dir =
			"/sys/devices/system/cpu/cpu0/cpufreq");
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
 * What the kernel does not do here. /dev/gpiomem bypasses the kernel's
 * pin ownership and cleanup:
 *  - Nothing stops another driver or process using the same pins. A pin
 *    whose function is an alternate one (SPI0's GPIO8-11, say) is refused,
 *    and so is any of the seven lines already an output at the start (a
 *    killed earlier load left it so): the message names them and gives the
 *    commands that put them back to inputs.
 *  - A GPFSEL register holds 10 pins (GPFSEL0 GPIO0-9, GPFSEL1 GPIO10-19,
 *    GPFSEL2 GPIO20-29). Changing one pin's function is a read-modify-write
 *    of its register, and another writer (a driver, another process)
 *    changing a different pin of the same register between our read and
 *    our write has its change lost. The window is tens of nanoseconds; it
 *    is not closed.
 *  - When the process dies without unwinding, the pins stay driven (CRESET_B
 *    low, the SPI lines out) until a later run, or pinctrl, puts them back.
 *    Handled and put back: errors, SIGINT, SIGTERM, SIGHUP, SIGQUIT and
 *    SIGPIPE (they unwind to the destructor). Not handled: SIGKILL,
 *    SIGSEGV, SIGBUS, SIGABRT, abort(), std::terminate(), exit() and
 *    _exit() called from elsewhere.
 *
 * Each pin's function (its GPFSEL bits) is saved when the load starts and
 * put back, and read back, by restore_functions(), which the destructor
 * runs. restore_functions() uses plain register accesses on a fixed array
 * (no allocation, no libc call but sigprocmask) and blocks the handled
 * signals around each read-modify-write, so it may be called from a signal
 * handler without that handler landing between another read and write.
 *
 * The SPI_SCK rate. The image is clocked by write() and clocks(), whose
 * loop is padded so that every SCK phase is at least 25 ns, every period at
 * least 50 ns (so at or under 20 MHz), against the data sheet's 20 ns
 * minimum SCK high and low time and 25 MHz maximum (DS-02008 Table 4.24
 * tCCLKH, tCCLKL, fMAX). The padding is calibrated once, in take_bus(), with
 * zero-mask writes (no pin moves), at the CPU clock cpufreq reports, and
 * scaled to the CPU's maximum clock: every cost measured is taken to be
 * CPU-bound, so at the maximum clock it is shorter by cur/max and the pad
 * is sized for that. The load also checks, from the same numbers scaled to
 * the minimum clock, that a bit takes 1000 ns or less there (1 MHz stays
 * reachable at the slowest clock); if not it stops. If cpufreq cannot be
 * read, the load stops: nothing is guessed.
 *
 * The assumption this rests on, and that no code here can check: register
 * writes are posted and reach the pins no sooner than the CPU issues them,
 * so spacing the CPU's writes spaces the pin edges at least as far apart
 * (the bus can delay a write, not hasten it). Whether the real pin edges
 * meet the 20 ns phases is measured by the hardware trial, not guaranteed
 * by this code. Ice40's per-chunk and overall rate checks stay the
 * authority on the mean, and see only the loop's time, not calibration.
 */
class GpiomemIce40Spi : public Ice40GpioSpi {
	public:
		/*!
		 * \param[in] regs: the GPIO registers (takes them over)
		 * \throws when a line is not GPIO 0-31, its function is an
		 *         alternate one, or it is an output already
		 */
		GpiomemIce40Spi(const ice40_gpio_pins_t &pins,
			std::unique_ptr<Ice40GpioRegs> regs, int8_t verbose);
		~GpiomemIce40Spi();

		void take_bus() override;
		void write(const uint8_t *data, uint32_t len) override;
		void clocks(uint32_t nclk) override;

		/*!
		 * \brief every pin changed so far back to the function it had
		 *        when the load started, and read back. Plain register
		 *        accesses only: no allocation, no exception, no libc call
		 *        but sigprocmask.
		 * \return how many pins did not read back as saved
		 */
		int restore_functions() noexcept;

		/*! \brief the spins of padding after each SPI_SCK edge now in use */
		uint32_t pad_spins() const { return _pad_spins; }
		/*!
		 * \brief the padding for a host whose three register writes of a
		 *        bit take bit_ns together and whose spin takes spin_ns, both
		 *        as at the CPU's maximum clock: the SCK high phase w + pad
		 *        is at least 25 ns (w = bit_ns / 3); the low phase 2w + pad
		 *        and the period 3w + 2 pad = 50 ns + w then follow
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
		 *        user named a gpiochip, or when `compatible` does not exist
		 *        (not a device-tree host).
		 * \param[in] chip_given: -d was given
		 * \param[in] compatible: /proc/device-tree/compatible
		 * \throws when `compatible` exists and cannot be read: the host
		 *         cannot be told, and nothing is guessed
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
		bool _calibrated;
};

#endif  // SRC_GPIOMEMICE40SPI_HPP_
