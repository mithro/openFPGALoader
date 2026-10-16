"""Build every commit of <base>..<tip> (in the current worktree), each by full sha."""
import subprocess, sys, pathlib, os
base, tip = sys.argv[1], sys.argv[2]
extra = sys.argv[3:]
here = pathlib.Path(__file__).parent
shas = subprocess.check_output(["git", "rev-list", "--reverse", f"{base}..{tip}"], text=True).split()
bad = 0
for s in shas:
    subj = subprocess.check_output(["git", "log", "-1", "--format=%s", s], text=True).strip()
    r = subprocess.run(["uv", "run", "--no-project", str(here / "build_check.py"), s, *extra], capture_output=True, text=True)
    print(f"{'OK  ' if r.returncode == 0 else 'FAIL'} {s[:12]} {subj}", flush=True)
    if r.returncode:
        bad += 1
        print(r.stdout[-2500:], r.stderr[-1000:], flush=True)
sys.exit(1 if bad else 0)
