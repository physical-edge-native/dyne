# Cooperative-run log: one format for the simulator and the devices

Date: 2026-11-26. Issue: #167. Decided by the maintainer (split into a simulator/analysis stage and a firmware stage;
on the devices, keep the log in memory and send it after the run); the format below was written with an AI assistant.

## Decision

1. A cooperative run is measured from a log with one JSON object per line. The simulator (`dyne-sim2d --perf-log`)
   writes it now; the firmware is to write the same format, so `scripts/coop_perf.py` analyses both and is checked
   without hardware.
2. On the devices the log is kept in memory during the run and sent after it ends (stage 2 of #167), so that sending
   the log does not disturb the traffic being measured (each device currently publishes about one MQTT message per
   second, #114).

## Format

| Record | Fields | Meaning |
|---|---|---|
| `{"k":"run", ...}` | `source`, `nodes`, `steps`; the simulator adds `step_us`, `lat_us`, `lat_jitter_us`, `loss`, `seed` | header, first line |
| `{"k":"tx", ...}` | `node`, `iface`, `step`, `t`, `bytes`, `resend` | a border message sent (resends and replies to duplicates included) |
| `{"k":"rx", ...}` | `node`, `iface`, `step`, `t_sent`, `t`, `bytes` | a border message received; `t_sent` is the sender's time (on the devices, the timestamp in the wire-format header) |
| `{"k":"step", ...}` | `node`, `step`, `t0`, `t1`, `t2` | a finished step: `t0` start (messages sent), `t1` all border values in, `t2` done |

Times are microseconds: virtual time in the simulator, synchronised clocks on the devices (so `t - t_sent` is a one-way
latency only as good as the clock synchronisation, see #154). `bytes` is the size of the delivery unit (the UDP payload
on the devices), without the envelope added on real radios.

## On the devices (stage 2)

- The configuration's `mode.perf_cap` (records, default 0 = off) turns recording on; the buffer is taken from PSRAM when
  the board has it (40 bytes per record; 300 steps on four links need about 3,000 records, 120 kB).
- After the run, before the `done` status, each device publishes its records to `dyne/node/<idx>/perf` in batches of
  `mode.perf_batch` records (default 20) every `mode.perf_pace_ms` (default 1000 ms), QoS 1, as
  `{"node","steps","i0","total","dropped","recs":[...],"n"}` with the record objects above. If the buffer fills, later
  records are dropped and counted.
- `scripts/coop_perf.py RECORDING --from-mqtt` reads an MQTT recording (`scripts/hw_replay.py record`), checks that every
  record from 0 to `total` arrived exactly once per device, and analyses the result as above.
- The record buffer, the JSON and the batches are C code without ESP-IDF (`node-esp32/main/dyne_perf_core.c`), checked
  on a PC (`node-esp32/host_test/perf_test.c`); the integration into `node_v2.c` is checked on the devices.

## Derived numbers (`scripts/coop_perf.py`)

One-way latency per link and overall (median, 95th and 99th percentile, maximum); jitter as the standard deviation and
as the mean absolute difference of consecutive latencies on a link; delivery ratio (received / sent); bytes and
messages sent per second; steps per second per device; step time split into waiting (`t1 - t0`) and computing
(`t2 - t1`). On the devices, "computing" also includes handing messages to the solver.

## Other options considered

- Send per-step records every N steps over MQTT: allows longer runs, but competes with the measured traffic.
- Log only aggregates (means per window), as the firmware's `rt` report does today: cannot give distributions or jitter.
- Separate formats for the simulator and the devices: the analysis could not be checked without hardware.
