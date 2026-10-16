# Restacking mithro/openFPGALoader

This branch holds no openFPGALoader code. It is the procedure and the scripts for keeping the fork's
fpgas.online branches current. The branches it maintains:

| Branch | What it is |
|---|---|
| `feature/<name>` | one feature: one to three upstream-quality patches on upstream `master` |
| `backport/v<release>/<name>` | the same feature on the latest upstream release tag |
| `fpgas-online/master` | octopus merge of every `feature/*` on upstream `master`, then one commit that puts the feature table at the top of README.md |
| `fpgas-online/v<release>` | the same for the backports, on the release tag |
| `master` | upstream `master`, nothing else |

fpgas-online/fpgas.online-fpga-tools pins the two integration branches in its `upstreams.toml` and builds the
pinned commits as they are; its daily bump moves the pins to the branch heads (and fails with `RESTACK` when
upstream has released a tag newer than `fpgas-online/v<release>` is built on).

Rules: nothing goes to trabucayre/openFPGALoader or any other upstream (no PR, issue or comment; `gh` defaults to
the parent in a fork, so always pass `--repo mithro/openFPGALoader`). Force pushes only with
`git safe-force-push origin <branch>`, and only to the branches above (never `master`). The feature list is
`scripts/features.py`.

## Setup (once per clone)

```sh
git clone git@github.com:mithro/openFPGALoader.git openfpgaloader && cd openfpgaloader
git remote add upstream https://github.com/trabucayre/openFPGALoader.git
git remote set-url --push upstream DISABLED-no-upstream-pushes
git config rerere.enabled true          # conflict resolutions are recorded and replayed
printf 'tmp/\n.worktrees/\n' >> .git/info/exclude
git worktree add .worktrees/restack fpgas-online/restack
docker build -t ofl-build-trixie   -f .worktrees/restack/scripts/Dockerfile.trixie   .worktrees/restack/scripts
docker build -t ofl-build-bookworm -f .worktrees/restack/scripts/Dockerfile.bookworm .worktrees/restack/scripts
```

librp1jtag for the `rp1pio` builds, once per suite (the static library must be built on the suite that links it):

```sh
mkdir -p tmp/rp1src && cd tmp/rp1src
git clone https://github.com/raspberrypi/utils.git && git clone https://github.com/mithro/rp1-jtag.git
cp ../../.worktrees/restack/scripts/build-rp1jtag.sh . && cd ..
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD:/tmpdir" ofl-build-trixie \
  sh /tmpdir/rp1src/build-rp1jtag.sh /tmpdir/rp1src/utils /tmpdir/rp1src/rp1-jtag /tmpdir/rp1prefix
# the same with ofl-build-bookworm and /tmpdir/rp1prefix-bookworm (remove rp1src/*/build first)
```

`S=.worktrees/restack/scripts`; run the scripts from a worktree of the branch being built.

## When upstream master moves

1. `git fetch upstream`. For each feature, in its own worktree:
   `git rebase upstream/master` on `feature/<name>`. Resolve conflicts so each patch stays one logical change with
   a message that is true of its code. Keep one to three patches.
2. Build every patch: `uv run --no-project $S/build_range.py upstream/master feature/<name>` (add
   `--rp1 rp1prefix` for `rp1pio-cable`). Every line must say OK.
3. Rebuild `fpgas-online/master` in a detached worktree:
   ```sh
   git checkout --detach upstream/master
   for f in flash-info netv2-board rp1pio-cable tt-fpga cmake-version-override ice40-spi-sram; do
     git merge --no-ff --no-edit feature/$f || break   # resolve, git add, git commit --no-edit, continue
   done
   uv run --no-project $S/octopus_commit.py upstream/master "upstream master ($(git rev-parse --short=7 upstream/master))" \
     "Conflicts are resolved by hand (recorded with git rerere)." fpgas-online/master master \
     feature/flash-info feature/netv2-board feature/rp1pio-cable feature/tt-fpga feature/cmake-version-override feature/ice40-spi-sram
   ```
   rerere replays earlier resolutions: read every file it resolved (`git diff HEAD` before committing); a replayed
   resolution can be wrong for new context.
4. Check the result: the integration's diff to upstream must be the union of the feature branches' diffs
   (`git diff upstream/master fpgas-online/master -- . ':!README.md'` against each `git diff upstream/master
   feature/<name>`); `prove_integration.py <base> fpgas-online/master upstream/master:feature/<name> ...` applies
   each feature's diff to the base and compares (README.md aside). Build it on trixie and bookworm with librp1jtag:
   `uv run --no-project $S/build_check.py fpgas-online/master trixie --rp1 rp1prefix`.
5. Independent review of every changed patch and of the integration (one change per patch, messages true, nothing
   fpgas.online-specific in the code, shortcuts hunted). Then `git safe-force-push origin <branch>` for each.
6. Nothing else: the next daily bump in fpgas.online-fpga-tools picks up the new `fpgas-online/master` head.

## When upstream releases a new tag

The daily bump fails with `RESTACK: fpgas-online/v<old> is built on v<old>, upstream released v<new>`. Then:
cherry-pick each `feature/<name>` onto `v<new>` as `backport/v<new>/<name>` (adapting to the release's code, same
messages unless the code differs), build each patch, merge them into `fpgas-online/v<new>` as in step 3 with base
`v<new>` and `$S/readme.py v<new>`, review, push, and change `[openfpgaloader.stable]` in fpgas.online-fpga-tools'
`upstreams.toml` to `ref = "fpgas-online/v<new>"` in a pull request there. `scripts/readme.py` and
`scripts/features.py` name the release in their links: update them for the new tag.

## Adding a feature

Branch `feature/<name>` from upstream `master` (one to three patches), add it to `scripts/features.py` (merge
order and table row), make its backport, then rebuild both integration branches as above.
