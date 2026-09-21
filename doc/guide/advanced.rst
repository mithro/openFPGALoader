.. _advanced-usage:

Advanced usage of openFPGALoader
################################

Resetting an FPGA
=================

.. code-block:: bash

    openFPGALoader [options] -r

Using negative edge for TDO's sampling
======================================

If transaction are unstable you can try to change read edge by using

.. code-block:: bash

    openFPGALoader [options] --invert-read-edge

Reading the bitstream from STDIN
================================

.. code-block:: bash

    cat /path/to/bitstream.ext | openFPGALoader --file-type ext [options]

``--file-type`` is required to detect file type.

.. NOTE::
  It's possible to load a bitstream through network:

  .. code-block:: bash

    # FPGA side
    nc -lp port | openFPGALoader --file-type xxx [option]

    # Bitstream side
    nc -q 0 host port < /path/to/bitstream.ext

Automatic file type detection bypass
====================================

Default behavior is to use file extension to determine file parser.
To avoid this mechanism ``--file-type type`` must be used.

FT231/FT232 bitbang mode and pins configuration
===============================================

FT232R and ft231X may be used as JTAG programmer.
JTAG communications are emulated in bitbang mode.

To use these devices user needs to provides both the cable and the pin mapping:

.. code-block:: bash

    openFPGALoader [options] -cft23XXX --pins=TDI:TDO:TCK:TMS /path/to/bitstream.ext

where:

* ft23XXX may be ``ft232RL`` or ``ft231X``.
* TDI:TDO:TCK:TMS may be the pin ID (0 <= id <= 7) or string value.

allowed values are:

===== ==
value ID
===== ==
 TXD  0
 RXD  1
 RTS  2
 CTS  3
 DTR  4
 DSR  5
 DCD  6
 RI   7
===== ==

Writing to an arbitrary address in flash memory
===============================================

With FPGA using an external SPI flash (*xilinx*, *lattice ECP5/nexus/ice40*, *anlogic*, *efinix*) option ``-o`` allows
one to write raw binary file to an arbitrary adress in FLASH.

Display detailed SPI flash information
======================================

``--flash-info`` works like ``--detect -f`` but also displays:

* manufacturer, part name and size (from the internal database, SFDP or the JEDEC ID),
* factory programmed unique ID / serial number when the manufacturer provides one
  (Winbond, GigaDevice, ISSI, Puya, Micron N25Q/MT25Q, Spansion S25FL128S/256S,
  Infineon S25FL-L, SST26),
* the JEDEC SFDP (JESD216) content: supported read modes (1-1-2, 1-2-2, 2-2-2,
  1-1-4, 1-4-4, 4-4-4) with opcodes and dummy clocks, DTR (DDR) support,
  4-Byte address and DTR read instructions, erase types, page size and
  quad enable method.

.. code-block:: bash

    openFPGALoader -b arty --flash-info

Only read-only commands are sent to the flash.

The report is not displayed, and openFPGALoader exits with an error, when
the JEDEC ID is not a valid one (manufacturer code failing JEP106 odd
parity) or is not the same when read twice: this happens when the flash
is not really reached (ie SPI bridge not loaded).

For scripts, ``--flash-info-json FILE`` does the same and also writes the
information to ``FILE`` as JSON. The file is only written when every flash
access succeeded (a previous ``FILE`` is removed at start), so an existing
file and a zero exit status mean the content was read:

.. code-block:: json

    {"format": "openFPGALoader-flash-info", "version": 1, "flashes": [
      {"jedec_id": "0x010219", "manufacturer_id": "0x01", "memory_type": "0x02",
       "capacity": "0x19", "manufacturer": "Spansion",
       "manufacturer_jep106": "Spansion / Cypress / Infineon",
       "part": "S25FL256S", "size_bytes": 33554432, "size_source": "database",
       "unique_id": {"state": "read", "value": "e2789916a0809a22bbc76634c1bf53ed",
                     "bits": 128, "opcode": "0x4b"},
       "sfdp": null}]}

``flashes`` has one entry per flash (two with ``--target-flash both``).
``unique_id.state`` is ``read``, ``blank`` (read, all ``0x00``/``0xFF``) or
``none`` (no known unique ID command for this part); a failed unique ID read
is an error. ``sfdp`` is ``null`` without SFDP, otherwise it holds the tables,
``bfpt`` (density, address mode, DTR, page size, read modes with opcode and
mode/dummy clocks, erase types, quad enable) and ``read_4byte``.

Detect/read/write on primary/secondary flash memories
=====================================================

With FPGA using two external SPI flash (some *xilinx* boards) option ``--target-flash`` allows to select the QSPI chip.

To detect:

.. code-block:: bash

    openFPGALoader -b kcu105 -f --target-flash {primary,secondary} --detect

To read the primary flash memory:

.. code-block:: bash

    openFPGALoader -b kcu105 --target-flash primary --dump-flash --file-size N_BYTES mydump.bin

and the second flash memory:

.. code-block:: bash

    openFPGALoader -b kcu105 --target-flash secondary --dump-flash --file-size N_BYTES --secondary-bitstream mydump.bin

To write on secondary flash memory:

.. code-block:: bash

    openFPGALoader -b kcu105 -f --target-flash secondary --secondary-bitstream mySecondaryBitstream.bin

Using an alternative directory for *spiOverJtag*
================================================

By setting ``OPENFPGALOADER_SOJ_DIR`` it's possible to override default
*spiOverJtag* bitstreams directory:

.. code-block:: bash

    export OPENFPGALOADER_SOJ_DIR=/somewhere
    openFPGALoader xxxx

or

.. code-block:: bash

    OPENFPGALOADER_SOJ_DIR=/somewhere openFPGALoader xxxx
