"""Put the feature table at the top of README.md (run in the integration worktree).

usage: readme.py master|v1.1.1
"""
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from features import REPO, FEATURES

kind = sys.argv[1]
if kind == "master":
    title = "upstream `master`"
    base_link = "https://github.com/trabucayre/openFPGALoader/tree/master"
    col = "Feature branch (on upstream `master`)"
    branch = lambda n: f"feature/{n}"
    other = f"The same features on the latest release, v1.1.1: [`fpgas-online/v1.1.1`]({REPO}/tree/fpgas-online/v1.1.1)."
else:
    title = "the v1.1.1 release"
    base_link = "https://github.com/trabucayre/openFPGALoader/tree/v1.1.1"
    col = "Backport branch (on v1.1.1)"
    branch = lambda n: f"backport/v1.1.1/{n}"
    other = f"The same features on upstream `master`: [`fpgas-online/master`]({REPO}/tree/fpgas-online/master)."

rows = []
for name, adds, what in FEATURES:
    b = branch(name)
    f, k = f"feature/{name}", f"backport/v1.1.1/{name}"
    rows.append(f"| {name} | {adds} | {what} | [`{f}`]({REPO}/tree/{f}) | [`{k}`]({REPO}/tree/{k}) |")
head = f"""<!-- fpgas.online feature table: generated, do not edit by hand -->
# openFPGALoader with fpgas.online features

This branch is [openFPGALoader]({base_link}) ({title}) with the features below merged in.
Each feature is its own branch of one to three patches, kept rebased on upstream `master`
and written to be suitable for upstream; each also has a backport on the latest release.
{other}

| Feature | Adds | What it does | On upstream `master` | On v1.1.1 |
|---|---|---|---|---|
""" + "\n".join(rows) + "\n<!-- end of fpgas.online feature table -->\n\n"
p = pathlib.Path("README.md")
t = p.read_text()
marker = "<!-- end of fpgas.online feature table -->\n\n"
if marker in t:
    t = t.split(marker, 1)[1]
p.write_text(head + t)
