// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 */

#ifndef SRC_ICE40SPISLAVEINTERFACE_HPP_
#define SRC_ICE40SPISLAVEINTERFACE_HPP_

#include <csignal>
#include <cstdint>
#include <stdexcept>
#include <string>

/*!
 * \file ice40SpiSlaveInterface.hpp
 * \class Ice40SpiSlaveInterface
 * \brief the lines an iCE40 SPI slave (peripheral) configuration needs:
 *        SPI_SS, SPI_SCK and SPI_SI driven by the host, CRESET_B driven
 *        by the host and CDONE read back (TN-02001 section 13).
 *
 * The SRAM load built on it is Ice40::program_cram_slave(): this interface
 * only moves lines, the procedure and its checks are the Ice40 class's.
 * Every method throws std::runtime_error when a line cannot be moved or
 * read: nothing is skipped.
 *
 * write(), clocks() and anything else that loops check abort_check() at
 * every bit: once the load's signal handler has set the flag given to
 * set_abort(), they stop at the next bit boundary and throw
 * Ice40SpiSlaveAborted.
 */
class Ice40SpiSlaveAborted: public std::runtime_error {
	public:
		explicit Ice40SpiSlaveAborted(int sig):
			std::runtime_error("interrupted by " + name(sig)), signal(sig) {}
		static std::string name(int sig) {
			switch (sig) {
			case SIGHUP: return "SIGHUP";
			case SIGQUIT: return "SIGQUIT";
			case SIGPIPE: return "SIGPIPE";
			case SIGINT: return "SIGINT";
			case SIGTERM: return "SIGTERM";
			default: return "signal " + std::to_string(sig);
			}
		}
		int signal;
};

class Ice40SpiSlaveInterface {
	public:
		Ice40SpiSlaveInterface(): _abort(NULL) {}
		virtual ~Ice40SpiSlaveInterface() {}

		/*!
		 * \brief the flag a signal handler sets (to the signal number) to
		 *        stop the load; NULL: none
		 */
		void set_abort(const volatile sig_atomic_t *flag) { _abort = flag; }
		/*!
		 * \brief throws Ice40SpiSlaveAborted once the flag is set
		 */
		void abort_check() const {
			if (abort_pending())
				throw Ice40SpiSlaveAborted(*_abort);
		}
		bool abort_pending() const { return _abort && *_abort; }
		/*!
		 * \brief the flag itself, for a loop too tight to call
		 *        abort_check(): never NULL, points at a flag that stays 0
		 *        when none was given
		 */
		const volatile sig_atomic_t *abort_flag() const {
			static const volatile sig_atomic_t none = 0;
			return _abort ? _abort : &none;
		}

		/*!
		 * \brief CRESET_B
		 * \param[in] hold: true: driven low (the iCE40 in reset, its SPI
		 *            pins released); false: let go (the board's pull-up
		 *            takes it high)
		 */
		virtual void hold_reset(bool hold) = 0;
		/*!
		 * \brief CDONE level
		 */
		virtual bool cdone() = 0;
		/*!
		 * \brief take the SPI bus: SPI_SS high, SPI_SCK low, SPI_SI high,
		 *        and whatever else shares the bus kept off it (a flash).
		 *        Called only with the iCE40 held in reset.
		 */
		virtual void take_bus() = 0;
		/*!
		 * \brief every line take_bus() drove is an input again;
		 *        throws (after trying them all) if one could not be
		 */
		virtual void release_bus() = 0;
		/*!
		 * \brief SPI_SS
		 * \param[in] high: true: deselected; false: selected
		 */
		virtual void set_ss(bool high) = 0;
		/*!
		 * \brief send len bytes, most-significant bit first, SPI mode 0:
		 *        SPI_SI changes while SPI_SCK is low, the iCE40 captures
		 *        it on the rising edge (TN-02001 13.2)
		 */
		virtual void write(const uint8_t *data, uint32_t len) = 0;
		/*!
		 * \brief nclk SPI_SCK cycles with SPI_SI high (dummy bits)
		 */
		virtual void clocks(uint32_t nclk) = 0;

	private:
		const volatile sig_atomic_t *_abort;
};

#endif  // SRC_ICE40SPISLAVEINTERFACE_HPP_
