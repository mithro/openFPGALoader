"""Build one openFPGALoader revision in a Debian container.

usage: build_check.py <rev> [<suite>] [--rp1 <prefix-in-repo-tmp>]
Exports <rev> with git archive, configures with every backend the tree knows,
builds, and runs --Version / --list-cables / --list-boards. Exit status is the
build's. Logs go to tmp/build/<rev>-<suite>.log.
"""
import argparse, pathlib, subprocess, sys, shutil, os

# the clone's tmp/ (gitignored or in .git/info/exclude) holds builds and librp1jtag prefixes
repo = pathlib.Path(subprocess.check_output(["git", "rev-parse", "--path-format=absolute",
    "--git-common-dir"], text=True).strip()).parent
ap = argparse.ArgumentParser()
ap.add_argument("rev")
ap.add_argument("suite", nargs="?", default="trixie")
ap.add_argument("--rp1", help="librp1jtag install prefix under the clone's tmp/ (built by build-rp1jtag.sh, see README)")
ap.add_argument("--git-dir", default=os.getcwd())
ap.add_argument("--keep", action="store_true", help="keep build/ for running the binary")
a = ap.parse_args()

sha = subprocess.check_output(["git", "-C", a.git_dir, "rev-parse", "--short=12", a.rev], text=True).strip()
out = repo / "tmp" / "build" / f"{sha}-{a.suite}"
if out.exists():
    shutil.rmtree(out)
out.mkdir(parents=True)
src = out / "src"
src.mkdir()
arch = subprocess.run(["git", "-C", a.git_dir, "archive", sha], check=True, capture_output=True).stdout
subprocess.run(["tar", "-x", "-C", str(src)], input=arch, check=True)
cm = (src / "CMakeLists.txt").read_text()
opts = ["-DCMAKE_BUILD_TYPE=Release", "-DENABLE_CABLE_ALL=ON", "-DENABLE_VENDORS_ALL=ON",
        "-DENABLE_LIBGPIOD=ON", "-DENABLE_UDEV=ON", "-DENABLE_USB_SCAN=ON"]
if "ENABLE_TT_MICROPYTHON" in cm:
    opts.append("-DENABLE_TT_MICROPYTHON=ON")
if "ENABLE_RP1_PIO" in cm:
    if a.rp1:
        opts += ["-DENABLE_RP1_PIO=ON", f"-DCMAKE_PREFIX_PATH=/tmpdir/{a.rp1}"]
    else:
        opts.append("-DENABLE_RP1_PIO=OFF")
if "OPENFPGALOADER_VERSION" in cm:
    opts.append("-DOPENFPGALOADER_VERSION=0.0.0-buildcheck")
script = (
    "set -e; cmake -S /work/src -B /work/build " + " ".join(opts) +
    " > /work/cmake.log 2>&1 || { cat /work/cmake.log; exit 1; }; "
    "grep -iE 'warn|not found|disabled' /work/cmake.log || true; "
    "cmake --build /work/build --parallel 8 -- -k 2>&1 | grep -E 'error|warning:|Error' || true; "
    "test -x /work/build/openFPGALoader; "
    "/work/build/openFPGALoader --Version; "
    "/work/build/openFPGALoader --list-cables > /work/cables.txt; "
    "/work/build/openFPGALoader --list-boards > /work/boards.txt; "
    "/work/build/openFPGALoader --help > /work/help.txt; echo BUILD-OK"
)
cmd = ["docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}",
       "-v", f"{out}:/work", "-v", f"{repo / 'tmp'}:/tmpdir:ro",
       f"ofl-build-{a.suite}", "sh", "-c", script]
r = subprocess.run(cmd, capture_output=True, text=True)
log = out.with_suffix(".log")
log.write_text(r.stdout + r.stderr)
ok = r.returncode == 0 and "BUILD-OK" in r.stdout
print(f"{a.rev} {sha} {a.suite}: {'OK' if ok else 'FAIL rc=' + str(r.returncode)}  log={log}")
if not ok:
    print((r.stdout + r.stderr)[-3000:])
if ok and not a.keep:
    shutil.rmtree(out / "build")
sys.exit(0 if ok else 1)
