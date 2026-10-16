"""Write the octopus merge commit for HEAD's tree (a sequential merge of the branches), then the README commit.

usage: octopus_commit.py <base> <base-desc> <proof-desc> <integration-branch> <readme-kind> <branch>...
"""
import subprocess, sys, pathlib
def git(*a, inp=None):
    r = subprocess.run(["git", *a], capture_output=True, text=True, input=inp)
    if r.returncode:
        sys.exit(f"git {a} failed: {r.stdout}{r.stderr}")
    return r.stdout.strip()
base, base_desc, proof, integ, kind, *branches = sys.argv[1:]
tree = git("rev-parse", "HEAD^{tree}")
for b in branches:  # every branch must already be merged into HEAD
    git("merge-base", "--is-ancestor", b, "HEAD")
names = "\n".join(f"- {b}" for b in branches)
msg = (f"Merge {len(branches)} feature branches onto {base_desc}\n\n{names}\n\n{proof}\n")
parents = sum((["-p", git("rev-parse", p)] for p in [base] + branches), [])
sha = git("commit-tree", tree, *parents, inp=msg)
git("checkout", "-q", "--detach", sha)
here = pathlib.Path(__file__).parent
subprocess.run(["uv", "run", "--no-project", str(here / "readme.py"), kind], check=True)
git("add", "README.md")
git("commit", "-q", "-m", "README: table of the features merged into this branch")
git("branch", "-f", integ, "HEAD")
print(integ, git("rev-parse", "HEAD"), "octopus", sha)
