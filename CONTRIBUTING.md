# Contributing to Dyne

Thank you for your interest. Dyne is a research prototype maintained by a
single author, so response times may vary (usually within a week).

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
4. Open a pull request describing what changed and why.

Issues and pull requests may be written in English or Japanese.

## Scope

In scope: the runtime, simulators, solvers compiled to WebAssembly, firmware,
and tools for experiments with many small devices. Ideas for new physics or
applications are welcome as issues; larger directions are discussed across the
[Physical-Edge Native Architecture](https://github.com/physical-edge-native) organization.

## Code of conduct

This project follows the organization-wide
[Code of Conduct](https://github.com/physical-edge-native/.github/blob/main/CODE_OF_CONDUCT.md).
