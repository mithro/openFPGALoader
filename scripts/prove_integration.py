"""Prove: target tree + sum of feature deltas (original -> new tip) == integration tree.

usage: prove_integration.py <target> <integration> <orig>:<new> ...
Applies each delta to a scratch worktree at <target>, then compares trees
(README.md excluded: the integration adds the table).
"""
import subprocess, sys, tempfile, os, pathlib
def git(*a, cwd=None, inp=None):
    r = subprocess.run(["git", *a], cwd=cwd, capture_output=True, text=True, input=inp)
    if r.returncode:
        sys.exit(f"git {a} failed: {r.stdout}{r.stderr}")
    return r.stdout
target, integ, *pairs = sys.argv[1:]
d = pathlib.Path(tempfile.mkdtemp(dir=os.environ["OFL_TMP"])) / "wt"
git("worktree", "add", "-q", "--detach", str(d), target)
try:
    for pair in pairs:
        orig, new = pair.split(":")
        patch = git("diff", "--binary", orig, new)
        if patch:
            git("apply", "--3way", "--index", cwd=d, inp=patch)
        print(f"applied {orig[:12]} -> {new[:12]}: {len(patch.splitlines())} lines")
    tree = git("write-tree", cwd=d).strip()
    diff = git("diff", "--stat", tree, integ, "--", ".", ":!README.md")
    print("DIFF vs integration (excluding README.md):", diff.strip() or "none")
    sys.exit(1 if diff.strip() else 0)
finally:
    git("worktree", "remove", "--force", str(d))
