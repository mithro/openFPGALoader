#!/bin/sh
# Build PIOLib (raspberrypi/utils piolib/) and librp1jtag as static archives
# and install them under a prefix, for linking into openFPGALoader and OpenOCD.
#
#   packaging/build-rp1jtag.sh <piolib-src> <rp1jtag-src> <prefix>
#
# Used by the static release builds (packaging/static/), whose binaries must
# run with no library beyond the kernel: only static archives are installed.
# The Debian packages use the shared librp1jtag0 instead (build-libs.sh).
# Works with glibc and musl (Alpine); needs cmake, a C compiler and pkg-config.
set -eu

piolib_src=$1
rp1jtag_src=$2
prefix=$3

# Accept either the raspberrypi/utils checkout or its piolib/ subdirectory.
if [ -f "$piolib_src/piolib/CMakeLists.txt" ]; then
	piolib_src="$piolib_src/piolib"
fi
[ -f "$piolib_src/CMakeLists.txt" ] || { echo "build-rp1jtag: no piolib CMakeLists.txt under $piolib_src" >&2; exit 1; }

# PIOLib. Only the library target: the piolib examples link against gpiolib
# from the sibling pinctrl/ tree, which is not built when piolib is configured
# on its own, so a full build fails at link time.
# CMAKE_INSTALL_LIBDIR=lib on both: GNUInstallDirs would otherwise pick a
# multiarch lib/<triplet> on Debian, and the consumers look in <prefix>/lib.
cmake -S "$piolib_src" -B "$piolib_src/build" \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX="$prefix" \
	-DCMAKE_INSTALL_LIBDIR=lib \
	-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
	-DBUILD_SHARED_LIBS=OFF
cmake --build "$piolib_src/build" --parallel --target pio
cmake --install "$piolib_src/build" --component piolib

# librp1jtag. CMake finds PIOLib through CMAKE_PREFIX_PATH; without it the
# library silently builds a stub backend, which is asserted against below.
cmake -S "$rp1jtag_src" -B "$rp1jtag_src/build" \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX="$prefix" \
	-DCMAKE_INSTALL_LIBDIR=lib \
	-DCMAKE_PREFIX_PATH="$prefix" \
	-DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build "$rp1jtag_src/build" --parallel
cmake --install "$rp1jtag_src/build"

# Keep the static archive only, so -lrp1jtag always resolves to it.
find "$prefix" -name 'librp1jtag.so*' -delete
find "$prefix" -name 'libpio.so*' -delete

archive=$(find "$prefix" -name 'librp1jtag.a' | head -n 1)
if [ -z "$archive" ]; then
	echo "build-rp1jtag: librp1jtag.a was not installed under $prefix" >&2
	exit 1
fi
if strings "$archive" | grep -q "compiled without PIOLib support"; then
	echo "build-rp1jtag: librp1jtag was built without PIOLib (stub backend)" >&2
	exit 1
fi
pc=$(find "$prefix" -name 'rp1jtag.pc' | head -n 1)
if [ -z "$pc" ]; then
	echo "build-rp1jtag: rp1jtag.pc was not installed under $prefix" >&2
	exit 1
fi
# Only the static archive is installed, and it calls PIOLib, so consumers
# that link -lrp1jtag through pkg-config must get -lpio as well. Upstream's
# .pc lists only -lrp1jtag (right for its shared library); amend the copy
# under this prefix rather than carrying a patch against rp1-jtag.
if ! grep -q -- '-lpio' "$pc"; then
	sed -i 's/^Libs: \(.*\)$/Libs: \1 -lpio -lpthread/' "$pc"
fi
grep -q -- '-lrp1jtag -lpio' "$pc" || { echo "build-rp1jtag: failed to amend $pc" >&2; exit 1; }
echo "build-rp1jtag: installed $archive and $pc ($(grep '^Libs:' "$pc"))"
