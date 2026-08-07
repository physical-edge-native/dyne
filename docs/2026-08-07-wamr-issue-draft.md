# WAMR issue draft (docs\2026-08-07-wamr-issue-draft.md)

Title: [xtensa/esp32] AOT module crashes (LoadProhibited) at first function call; identical crash state across different LLVM backends suggests loader-side relocation issue

## Environment

- Runtime: `espressif/wasm-micro-runtime` ESP-IDF component **2.4.0~1** (WAMR 2.4.0),
  ESP-IDF v5.5.5, target **esp32** (ESP32-WROOM-32, no PSRAM), classic interpreter
  and AOT enabled, app runs inside a pthread.
- wamrc: **2.4.0**, built from tag WAMR-2.4.0 with `build_llvm.py --platform xtensa`
  (espressif/llvm-project `xtensa_release_18.1.2`, and separately `xtensa_release_19.1.2`).
- Module: 2,181-byte `no_std` Rust wasm (wasm32-unknown-unknown, rustc 1.91,
  `-zstack-size=32768`, min memory 2 pages). No imports; exports plain C-ABI
  functions (i32/f64 params). Runs correctly on-device under the classic
  interpreter (2000-step FDM heat solve, results bit-identical to PC).

## Steps

1. `wamrc --target=xtensa --cpu=esp32 --bounds-checks=1 -o m.aot m.wasm`
2. Embed `m.aot`, load + instantiate on esp32 (both succeed;
   `wasm_runtime_load` / `wasm_runtime_instantiate` return OK, AOT code is
   placed in IRAM heap).
3. Call the first exported function via `wasm_runtime_call_wasm_a`.

## Result

Immediate `LoadProhibited` inside the AOT-generated code:

```
PC 0x4008ce1a (IRAM, inside AOT code)  EXCCAUSE 0x1c
A8  0x81008136        EXCVADDR 0x81008146  (= A8 + 0x10)
A2  0x3ffe46c0 (exec_env, valid)  A3 0x41 (= first i32 arg, correct)
backtrace: invokeNative_xtensa.s:65 -> aot code
```

Argument passing works (A3 holds the first argument); the generated code then
loads a garbage base pointer and dereferences `base + 0x10`.

## Matrix of experiments (all on the same board/firmware)

| aot generation | size | outcome |
|---|---|---|
| LLVM 18.1.2, O3 | 7,776 B | crash above |
| + `--cpu=esp32 --bounds-checks=1` | 7,776 B (identical file) | n/a (same defaults) |
| + `--enable-indirect-mode --disable-llvm-intrinsics` | 6,184 B | loader rejects: "cannot apply relocation to text section for aot file generated with --enable-indirect-mode" (wamrc had warned: 78 text relocations remain) |
| `--opt-level=0` | 11,124 B | same-pattern crash (garbage base 0xbf14e528, deref +0x10) |
| **LLVM 19.1.2**, O3 | 7,692 B | **bit-identical crash state to row 1** (same PC 0x4008ce1a, same A8 0x81008136, same EXCVADDR) |

Control: the same wasm compiled with the same wamrc build for `--target=i386
--bounds-checks=1` runs perfectly under a WAMR 2.4.0 linux x86_32 iwasm
(2000 steps, results identical to interpreter).

## Why we suspect the runtime loader rather than codegen

Two AOT files produced by *different LLVM backends* (18.1.2 vs 19.1.2, different
file sizes/bytes) crash with a *bit-identical* machine state. The compiler side
was varied five ways; the only invariant is the on-device AOT loader/relocation
path (aot_reloc_xtensa / literal handling) of component 2.4.0~1. The symptom
(consistent garbage constant being used as a base pointer, then `+0x10`
dereference) matches an unrelocated/mis-relocated literal being loaded by
`l32r`-style code.

Also: interpreter (classic) executes the same module correctly on the same
firmware, and fast-interp of this component fails to load any module
("allocate memory failed" with ~247 KB free heap) — reported separately.

## Ask

- Is xtensa AOT expected to work on esp32 with component 2.4.0~1?
- Are there known fixes in `aot_reloc_xtensa.c` / xtensa literal placement after
  WAMR-2.4.0 that we should try?
- If useful we can share both .aot files, the wasm, and full panic dumps.