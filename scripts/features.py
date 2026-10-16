"""The features carried by mithro/openFPGALoader, in merge order."""
REPO = "https://github.com/mithro/openFPGALoader"
FEATURES = [
    ("flash-info", "`--flash-info`, `--flash-info-json`",
     "Read-only SPI flash report: JEDEC and extended ID, factory unique ID, "
     "JESD216 SFDP (read modes, erase types, 4-byte instructions), as text or a JSON file; "
     "a failed flash detection exits non-zero."),
    ("netv2-board", "boards `netv2`, `netv2_100`",
     "NeTV2 (Artix-7 35T / 100T) on a Raspberry Pi header; the GPIO cable is chosen at run time "
     "(`rp1pio` on a Pi 5, else `libgpiod`)."),
    ("rp1pio-cable", "cable `rp1pio`",
     "JTAG through the Raspberry Pi 5 RP1 PIO block (librp1jtag), (a 3.8 MB Artix-7 bitstream in about 6.5 s, against 40 s with libgpiod). "
     "CMake option `ENABLE_RP1_PIO`."),
    ("tt-fpga", "board `tt_fpga`, cable `tt_micropython`",
     "TinyTapeout FPGA Demo Board (TTDBv3): iCE40UP5K SRAM load through the RP2040/RP2350's MicroPython raw REPL; "
     "the project clock on GPIO16 at 50 MHz. CMake option `ENABLE_TT_MICROPYTHON`."),
    ("cmake-version-override", "CMake `OPENFPGALOADER_VERSION`",
     "Lets a packager set the version string `--Version` reports."),
    ("ice40-spi-sram", "board `fomu_evt_header`",
     "iCE40 SRAM load over any SPI slave interface (TN-02001), on host GPIO lines through libgpiod, "
     "or through /dev/gpiomem registers on a Raspberry Pi 1-4."),
]


if __name__ == "__main__":
    # the merge order, one name per line (README, step 3)
    print("\n".join(name for name, _, _ in FEATURES))
