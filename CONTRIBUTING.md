# Contributing to Dyne

Thank you for your interest. Dyne is a research prototype maintained by a
single author, so response times may vary (usually within a week).

## Your first contribution

New here? Welcome. Start with one of these:

- **Good first issues** (small, well-defined):
  https://github.com/physical-edge-native/dyne/labels/good%20first%20issue
- **Help wanted** (ESP32 boards, firmware, numerics; bigger):
  https://github.com/physical-edge-native/dyne/labels/help%20wanted

Each issue says whether it needs hardware. Without hardware you can work on the simulators, solvers, scripts,
tests and documentation; hardware issues name the boards they need.

**Before you start, comment on the issue** ("I'd like to work on this") so two people do not do the same thing.

**How to make the pull request**: follow the
[step-by-step guide of the organization](https://github.com/physical-edge-native/.github/blob/main/CONTRIBUTING.md#step-by-step-your-first-pull-request).
For pull requests from forks, CI starts only after the maintainer approves it.

**Checks in this repository**: `bash scripts/ci-check.sh` from the repository root runs everything CI runs
(needs Rust stable with the `wasm32-unknown-unknown` target and Python 3.8+; it skips the parts whose Python
packages are missing). To run only one Rust test: `cargo test --release -p <crate> --test <name>`.

**Hardware records**: never commit Wi-Fi names or passwords (put them in `node-esp32/sdkconfig.local`, which Git
ignores). In logs and records, replace each board's MAC address with `02:00:00:00:02:YY` (`YY` = the last byte of
the real address, which the firmware uses as the node number) and the access point's with `02:00:00:00:00:01`.

**Questions**: in the issue, in [GitHub Discussions](https://github.com/physical-edge-native/dyne/discussions),
or on Discord `#dyne` (invite link in the organization's
[SUPPORT.md](https://github.com/physical-edge-native/.github/blob/main/SUPPORT.md)).
You may use AI coding assistants; say so in the pull request (see the organization guide).

## Reporting problems and asking questions

Open a [GitHub issue](https://github.com/physical-edge-native/dyne/issues).
Please include your OS, Rust version (`rustc --version`), the exact command,
and its output. For hardware problems, also include the board and ESP-IDF version.

## Proposing changes

1. Open an issue first for anything larger than a small fix, so we can agree on the approach.
2. Fork, create a branch, and make your change.
3. Run `bash scripts/ci-check.sh`. All checks must pass: numerical results are
   compared **bit for bit**, so any change in output is treated as a regression
   unless it is intended and explained in the pull request.
4. Add one changelog fragment `changelog.d/<issue>.<section>.md` (see
   [changelog.d/README.md](changelog.d/README.md)); do not edit `CHANGELOG.md` directly.
5. Open a pull request describing what changed and why, with `Closes #<issue>`.

Issues and pull requests may be written in English or Japanese.

## Scope

In scope: the runtime, simulators, solvers compiled to WebAssembly, firmware,
and tools for experiments with many small devices. Ideas for new physics or
applications are welcome as issues; larger directions are discussed across the
[Physical-Edge Native Architecture](https://github.com/physical-edge-native) organization.

## Code of conduct

This project follows the organization-wide
[Code of Conduct](https://github.com/physical-edge-native/.github/blob/main/CODE_OF_CONDUCT.md).

## License of contributions

Dyne is licensed under either of MIT or Apache-2.0, at your option (see [README](README.md#license)).
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in this work
by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
