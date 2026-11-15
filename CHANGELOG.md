# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project uses [Semantic Versioning](https://semver.org/) (0.x: interfaces may change).

## [Unreleased]

Entries for the next release are collected in [`changelog.d/`](changelog.d/README.md).

## [0.1.0] - 2026-11-15

First public release. Development history since 2026-08 is included.

### Added
- WebAssembly solvers for the 1D/2D heat equation and a learned 1D solver
  (time-discrete physics-informed neural network).
- Deterministic N-device simulators (1D, 2D) with injected packet loss, delay,
  pauses and partitions; split solutions are bit-exact with single-device solutions.
- Adding, removing and handing over devices at runtime.
- ESP32 firmware running the same WebAssembly solvers over Wi-Fi with
  configuration and monitoring over MQTT, and on-device data assimilation.
- Room-shape reconstruction from range sensors (simulator stage).
- Cross-platform regression script (`scripts/ci-check.sh`) and CI.

### Changed
- Wi-Fi scan logs in `measured/` have MAC addresses replaced by pseudonyms
  (consistent within the data set) before publication.
