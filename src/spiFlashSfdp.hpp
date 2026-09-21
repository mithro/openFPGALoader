// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 *
 * Serial Flash Discoverable Parameters (JEDEC JESD216) parser
 */

#ifndef SRC_SPIFLASHSFDP_HPP_
#define SRC_SPIFLASHSFDP_HPP_

#include <stdint.h>

#include <functional>
#include <string>
#include <vector>

/* SFDP parameter header (one per parameter table) */
typedef struct {
	uint16_t id;      /**< (MSB << 8) | LSB, 0xFF00 = BFPT */
	uint8_t major;
	uint8_t minor;
	uint8_t length;   /**< table length in DWORDs */
	uint32_t ptp;     /**< table address */
} sfdp_param_header_t;

/* one read mode (x-y-z = instruction-address-data line count) */
typedef struct {
	std::string name;     /**< ie "1-1-4" */
	uint8_t opcode;
	uint8_t mode_clocks;
	uint8_t dummy_clocks;
} sfdp_read_mode_t;

/* one erase type (from BFPT DWORD 8/9) */
typedef struct {
	uint32_t size;        /**< in Byte */
	uint8_t opcode;
} sfdp_erase_type_t;

/* one 4-Byte address read instruction (4BAIT) */
typedef struct {
	std::string name;
	uint8_t opcode;
} sfdp_4b_instr_t;

class SFDP {
	public:
		/*!
		 * \brief SFDP read callback: read len Byte at SFDP address addr
		 * \return false when read fails
		 */
		typedef std::function<bool(uint32_t addr, uint8_t *buf,
				uint32_t len)> read_fn_t;

		SFDP():_valid(false), _major(0), _minor(0) {}

		/*!
		 * \brief read and parse SFDP structures
		 * \param[in] rd: SFDP read callback
		 * \return false when no (valid) SFDP header is found
		 */
		bool parse(read_fn_t rd);

		bool valid() const { return _valid; }
		uint8_t major() const { return _major; }
		uint8_t minor() const { return _minor; }
		const std::vector<sfdp_param_header_t> &headers() const { return _headers; }
		const std::vector<uint32_t> &bfpt() const { return _bfpt; }
		bool has_bfpt() const { return _bfpt.size() >= 9; }

		/* BFPT decoded content (only meaningful when has_bfpt()) */
		uint64_t density_bits() const;
		/* 0: 3B only, 1: 3B or 4B, 2: 4B only, 3: reserved */
		uint8_t address_bytes() const;
		bool dtr_support() const;
		/* read modes, 1-1-1 (0x03) always first */
		std::vector<sfdp_read_mode_t> read_modes() const;
		std::vector<sfdp_erase_type_t> erase_types() const;
		/* 0 if not provided (BFPT < 11 DWORDs) */
		uint32_t page_size() const;
		/* Quad Enable Requirements description, empty if not provided */
		std::string quad_enable_req() const;
		/* 4-Byte Address Instruction Table read instructions */
		std::vector<sfdp_4b_instr_t> read_4b_instr() const;
		bool has_4bait() const { return !_4bait.empty(); }

		/*!
		 * \brief human readable name of a parameter table ID
		 * \param[in] id: (MSB << 8) | LSB
		 * \param[in] mfr_id: flash JEDEC manufacturer ID: a table whose
		 *            LSB is this ID is a vendor table
		 */
		static std::string table_name(uint16_t id, uint8_t mfr_id);

		/* display everything on stdout */
		void display(uint8_t mfr_id) const;

	private:
		uint32_t dw(uint8_t index) const {  // 1-based as in JESD216
			return (index <= _bfpt.size()) ? _bfpt[index - 1] : 0;
		}
		bool _valid;
		uint8_t _major;
		uint8_t _minor;
		std::vector<sfdp_param_header_t> _headers;
		std::vector<uint32_t> _bfpt;
		std::vector<uint32_t> _4bait;
};

#endif  // SRC_SPIFLASHSFDP_HPP_
