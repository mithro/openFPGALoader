// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2026 Tim 'mithro' Ansell <me@mith.ro>
 *
 * Serial Flash Discoverable Parameters (JEDEC JESD216) parser
 *
 * Bit positions follow JESD216 (Basic Flash Parameter Table and
 * 4-Byte Address Instruction Table), cross-checked with the Linux
 * kernel drivers/mtd/spi-nor/sfdp.c implementation.
 */

#include <stdio.h>

#include <string>
#include <vector>

#include "spiFlashSfdp.hpp"

#define SFDP_SIGNATURE   0x50444653  /* "SFDP" */
#define SFDP_BFPT_ID     0xFF00
#define SFDP_4BAIT_ID    0xFF84
/* sanity limits */
#define SFDP_MAX_HEADERS 32
#define SFDP_MAX_DWORDS  64

/* BFPT DWORD 1 */
#define BFPT_DW1_READ_1_1_2      (1 << 16)
#define BFPT_DW1_ADDR_BYTES(x)   (((x) >> 17) & 0x03)
#define BFPT_DW1_DTR             (1 << 19)
#define BFPT_DW1_READ_1_2_2      (1 << 20)
#define BFPT_DW1_READ_1_4_4      (1 << 21)
#define BFPT_DW1_READ_1_1_4      (1 << 22)
/* BFPT DWORD 5 */
#define BFPT_DW5_READ_2_2_2      (1 << 0)
#define BFPT_DW5_READ_4_4_4      (1 << 4)

static uint32_t le32(const uint8_t *b)
{
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
		((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static bool read_table(SFDP::read_fn_t rd, const sfdp_param_header_t &hdr,
		std::vector<uint32_t> &table)
{
	uint8_t len = hdr.length;
	if (len == 0)
		return false;
	if (len > SFDP_MAX_DWORDS)
		len = SFDP_MAX_DWORDS;
	std::vector<uint8_t> buf(len * 4);
	if (!rd(hdr.ptp, buf.data(), len * 4))
		return false;
	table.clear();
	for (int i = 0; i < len; i++)
		table.push_back(le32(&buf[i * 4]));
	return true;
}

bool SFDP::parse(read_fn_t rd)
{
	uint8_t hdr[8];

	_valid = false;
	_headers.clear();
	_bfpt.clear();
	_4bait.clear();

	if (!rd(0, hdr, 8) || le32(hdr) != SFDP_SIGNATURE)
		return false;

	_minor = hdr[4];
	_major = hdr[5];
	int nph = hdr[6] + 1;
	if (nph > SFDP_MAX_HEADERS)
		nph = SFDP_MAX_HEADERS;

	std::vector<uint8_t> ph(nph * 8);
	if (!rd(8, ph.data(), nph * 8))
		return false;

	for (int i = 0; i < nph; i++) {
		const uint8_t *p = &ph[i * 8];
		sfdp_param_header_t h;
		h.id = (uint16_t)((p[7] << 8) | p[0]);
		h.minor = p[1];
		h.major = p[2];
		h.length = p[3];
		h.ptp = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16);
		_headers.push_back(h);
	}

	_valid = true;

	/* the first header is always the BFPT (JESD216), but JESD216 rev 1.0
	 * parts may report MSB as 0x00: match on LSB for the first header,
	 * and keep the latest BFPT revision if several are present
	 */
	const sfdp_param_header_t *bfpt_hdr = NULL;
	for (size_t i = 0; i < _headers.size(); i++) {
		const sfdp_param_header_t &h = _headers[i];
		if ((h.id == SFDP_BFPT_ID || (i == 0 && (h.id & 0xff) == 0x00)) &&
				(!bfpt_hdr || h.major > bfpt_hdr->major ||
				 (h.major == bfpt_hdr->major && h.minor >= bfpt_hdr->minor)))
			bfpt_hdr = &h;
	}
	if (bfpt_hdr)
		read_table(rd, *bfpt_hdr, _bfpt);

	for (const auto &h : _headers) {
		if (h.id == SFDP_4BAIT_ID) {
			read_table(rd, h, _4bait);
			break;
		}
	}

	return true;
}

uint64_t SFDP::density_bits() const
{
	const uint32_t d = dw(2);
	if (d & 0x80000000) {
		const uint32_t n = d & 0x7fffffff;
		return (n < 64) ? (1ULL << n) : 0;
	}
	return (uint64_t)d + 1;
}

uint8_t SFDP::address_bytes() const
{
	return BFPT_DW1_ADDR_BYTES(dw(1));
}

bool SFDP::dtr_support() const
{
	return (dw(1) & BFPT_DW1_DTR) != 0;
}

/* opcode/mode/dummy descriptor stored in a 16 bits half DWORD */
static sfdp_read_mode_t read_mode(const std::string &name, uint32_t dword,
		int shift)
{
	const uint16_t v = (uint16_t)(dword >> shift);
	sfdp_read_mode_t m;
	m.name = name;
	m.dummy_clocks = v & 0x1f;
	m.mode_clocks = (v >> 5) & 0x07;
	m.opcode = (v >> 8) & 0xff;
	return m;
}

std::vector<sfdp_read_mode_t> SFDP::read_modes() const
{
	std::vector<sfdp_read_mode_t> modes;
	modes.push_back({"1-1-1", 0x03, 0, 0});
	if (!has_bfpt())
		return modes;

	const uint32_t dw1 = dw(1), dw5 = dw(5);
	if (dw1 & BFPT_DW1_READ_1_1_2)
		modes.push_back(read_mode("1-1-2", dw(4), 0));
	if (dw1 & BFPT_DW1_READ_1_2_2)
		modes.push_back(read_mode("1-2-2", dw(4), 16));
	if (dw5 & BFPT_DW5_READ_2_2_2)
		modes.push_back(read_mode("2-2-2", dw(6), 16));
	if (dw1 & BFPT_DW1_READ_1_1_4)
		modes.push_back(read_mode("1-1-4", dw(3), 16));
	if (dw1 & BFPT_DW1_READ_1_4_4)
		modes.push_back(read_mode("1-4-4", dw(3), 0));
	if (dw5 & BFPT_DW5_READ_4_4_4)
		modes.push_back(read_mode("4-4-4", dw(7), 16));
	return modes;
}

std::vector<sfdp_erase_type_t> SFDP::erase_types() const
{
	std::vector<sfdp_erase_type_t> types;
	if (!has_bfpt())
		return types;
	const uint32_t raw[2] = {dw(8), dw(9)};
	for (int i = 0; i < 4; i++) {
		const uint16_t v = (uint16_t)(raw[i / 2] >> ((i % 2) * 16));
		const uint8_t size_exp = v & 0xff;
		if (size_exp == 0 || size_exp >= 32)
			continue;
		types.push_back({1U << size_exp, (uint8_t)(v >> 8)});
	}
	return types;
}

uint32_t SFDP::page_size() const
{
	if (_bfpt.size() < 11)
		return 0;
	return 1U << ((dw(11) >> 4) & 0x0f);
}

std::string SFDP::quad_enable_req() const
{
	if (_bfpt.size() < 15)
		return "";
	switch ((dw(15) >> 20) & 0x07) {
		case 0: return "no QE bit (quad mode selected by instruction)";
		case 1: return "QE = SR2 bit 1, write SR1+SR2 with 0x01 "
			"(1 Byte write clears SR2)";
		case 2: return "QE = SR1 bit 6, write with 0x01";
		case 3: return "QE = SR2 bit 7, write 0x3E / read 0x3F";
		case 4: return "QE = SR2 bit 1, write SR1+SR2 with 0x01";
		case 5: return "QE = SR2 bit 1, read 0x35, write SR1+SR2 with 0x01";
		case 6: return "QE = SR2 bit 1, read 0x35, write 0x31";
		default: return "reserved";
	}
}

std::vector<sfdp_4b_instr_t> SFDP::read_4b_instr() const
{
	static const struct { int bit; const char *name; uint8_t op; } list[] = {
		{0,  "1-1-1 read",      0x13},
		{1,  "1-1-1 fast read", 0x0C},
		{2,  "1-1-2 fast read", 0x3C},
		{3,  "1-2-2 fast read", 0xBC},
		{4,  "1-1-4 fast read", 0x6C},
		{5,  "1-4-4 fast read", 0xEC},
		{20, "1-1-8 fast read", 0x7C},
		{21, "1-8-8 fast read", 0xCC},
		{13, "1-1-1 DTR read",  0x0E},
		{14, "1-2-2 DTR read",  0xBE},
		{15, "1-4-4 DTR read",  0xEE},
	};
	std::vector<sfdp_4b_instr_t> instr;
	if (_4bait.empty())
		return instr;
	for (const auto &e : list)
		if (_4bait[0] & (1U << e.bit))
			instr.push_back({e.name, e.op});
	return instr;
}

std::string SFDP::table_name(uint16_t id)
{
	switch (id) {
		case 0xFF00: return "Basic Flash Parameter Table";
		case 0xFF81: return "Sector Map";
		case 0xFF84: return "4-Byte Address Instruction";
		case 0xFF87: return "Status, Control and Configuration Register Map";
		default: break;
	}
	if ((id >> 8) == 0xFF)
		return "JEDEC table";
	return "vendor table";
}

static std::string human_size(uint64_t bytes)
{
	char buf[64];
	if (bytes >= (1ULL << 20) && (bytes % (1ULL << 20)) == 0)
		snprintf(buf, sizeof(buf), "%llu MiB", (unsigned long long)(bytes >> 20));
	else if (bytes >= 1024 && (bytes % 1024) == 0)
		snprintf(buf, sizeof(buf), "%llu KiB", (unsigned long long)(bytes >> 10));
	else
		snprintf(buf, sizeof(buf), "%llu B", (unsigned long long)bytes);
	return buf;
}

void SFDP::display() const
{
	if (!_valid) {
		printf("SFDP              : not supported\n");
		return;
	}
	printf("SFDP revision     : %u.%u\n", _major, _minor);
	for (const auto &h : _headers)
		printf("  table 0x%04x v%u.%u %3u DWORDs @ 0x%06x : %s\n",
				h.id, h.major, h.minor, h.length, h.ptp,
				table_name(h.id).c_str());
	if (!has_bfpt()) {
		printf("BFPT              : missing or truncated\n");
		return;
	}

	const uint64_t bits = density_bits();
	printf("Density           : %llu Mbit (%s)\n",
			(unsigned long long)(bits >> 20), human_size(bits / 8).c_str());

	static const char *addr_modes[] = {"3-Byte only", "3-Byte or 4-Byte",
		"4-Byte only", "reserved"};
	printf("Address mode      : %s\n", addr_modes[address_bytes()]);
	printf("DTR (DDR) clocking: %s\n", dtr_support() ? "supported" : "no");

	const uint32_t page = page_size();
	if (page)
		printf("Page size         : %u Byte\n", page);

	printf("Read modes (instruction-address-data lines):\n");
	for (const auto &m : read_modes())
		printf("  %-6s opcode 0x%02X  mode clocks %u  dummy clocks %u\n",
				m.name.c_str(), m.opcode, m.mode_clocks, m.dummy_clocks);

	if (has_4bait()) {
		printf("4-Byte address read instructions:\n");
		for (const auto &i : read_4b_instr())
			printf("  %-16s opcode 0x%02X\n", i.name.c_str(), i.opcode);
	}

	printf("Erase types:\n");
	for (const auto &e : erase_types())
		printf("  %-8s opcode 0x%02X\n", human_size(e.size).c_str(), e.opcode);

	const std::string qer = quad_enable_req();
	if (!qer.empty())
		printf("Quad enable       : %s\n", qer.c_str());
}
