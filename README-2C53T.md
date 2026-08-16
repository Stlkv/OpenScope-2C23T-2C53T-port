# Experimental 2C53T port (FNIRSI 2C53T target)

This branch ports the F2C23T firmware to the **FNIRSI 2C53T** and adds an
experimental FPGA transport that produces **live oscilloscope waveforms on both
channels** from a cold boot. Findings and open questions are tracked in
[OpenScope-2C53T issue #18](https://github.com/DavidClawson/OpenScope-2C53T/issues/18).

The bench journal — every measurement, the numbers behind each conclusion, and
the negative results — is in [docs/EXPERIMENT-LOG.md](docs/EXPERIMENT-LOG.md).
Newest sections first; read the top two for the current state of the capture
path and the vertical scale.

## What works on the 2C53T

- Full UI with all 15 buttons (4x3 matrix + 3 passive, scan ported from
  OpenScope-2C53T `button_scan.c`); SELECT→F2, TRIGGER→F3, PRM→F4
- Power (PC9 hold), backlight (PB8), battery (PB1/ADC ch9)
- **Live CH1 waveform** after a warm handoff (see below); Vpp measurements
- On-screen transport telemetry (two white lines in the scope grid)

Working as of 2026-08-16: both channels independently (CH2's relay bank had
been sitting on the attenuated path), the timebase knob over the engine's own
sample-rate ladder from 5 us/div to 5 ms/div plus roll, and the volts/div knob
over the relay ladder with volts computed from a measured scale.

Not working yet: absolute vertical calibration (readings run 7-10% high against
a generator's dial and want a proper reference), the signal generator, the
buzzer, and the 500 us/div to 20 ms/div timebase gap.

## Build

Requires clang and `ld.lld` (any LLVM lld; on macOS `brew install lld`,
or use rust-lld from a Rust toolchain).

```sh
make release-2c53t RUST_LLD=$(which ld.lld)
# -> dist/F2C23T-<version>-2C53T-08007000.bin
```

## Flash / recovery (no case opening, factory bootloader untouched)

Flash via the stock IAP channel: power off → hold MENU → press power →
device mounts an `IAP` disk → copy the `.bin` (Windows) or use
OpenScope-2C53T's `scripts/iap_flash.py` (macOS/Linux; do NOT drag-drop in
Finder). Restore stock the same way with FNIRSI's `APP_2C53T_V1.2.0_251015.bin`.

## Warm handoff (required for live capture)

The Gowin FPGA keeps its SRAM configuration as long as board power never
drops, and this port deliberately performs **no FPGA configuration** —
it inherits stock's:

1. Flash stock, boot it, open the oscilloscope (FPGA gets configured).
2. Keep USB connected, battery charged. **Do not unplug anything.**
3. Hold MENU + poke the pinhole reset (NRST) — the MCU resets into the
   factory IAP without a board power cycle.
4. Flash this firmware. It boots straight into the scope; feed a signal
   into CH1.

Any power loss (including a USB replug) reverts the FPGA to its NV design
and reads return 0xFF — redo from step 1. Custom→custom reflash iterations
keep the state, so the edit-flash-test loop takes ~2 minutes.

Key protocol facts encoded in `src/fpga.c` (HW_TARGET_2C53T section):
SPI3 mode 3 / soft CS PB6 / PC6+PB11 HIGH / PC0 data-ready; per-channel
1026-byte read windows (opcode 0x04/0x05 + 2 status bytes + 1023 samples,
ADC offset −28); the scope trigger reference is MCU DAC1 (PA4,
`DHR12R1 @ 0x40007408`) and MUST be restored after reset — a zeroed DAC1
yields all-zero captures.

## Telemetry (top of the scope grid)

```
line 1:  I<init> P<PC0 now> R<reads> C<forced> S<status:4=frame> F<frames> W<waits>
line 2:  <r0r1r2 of CH1 window> <CH1 min-max> b<CH2 min-max> D<windows identical>
         A<sweep byte><. searching | ! latched> Q<0x03 status reply> X<relay bank>Y<gain bank>
```

`min-max` spread of 2–4 counts = noise floor (no signal); tens = live signal.
In scope mode, PRM cycles 16 patterns of PC12/PE4/PE5/PE6 (X), TRIGGER
cycles PA15/PA10/PB9/PA6 (Y) — analog-frontend experiments.

## Experiment build flags (see `src/fpga.c`)

| Flag | Effect |
|---|---|
| `FPGA53_SEND_CFG=1` | send stock's five post-config SPI3 writes at init |
| `FPGA53_CFG02_VAL=0xNN` | value for the `02` write (acquisition-mode register; `03`=runs, most others stall) |
| `FPGA53_PRE_CMD=1` | send `0x83` preamble before each frame |
| `FPGA53_SWEEP_PRECMD=1` | auto-sweep preamble 0x80–0xFF, latch on CH2 spread |
| `FPGA53_SWEEP_CFG02=1` | auto-sweep the `02` register value |
| `FPGA53_DUAL_READ=1` | single-window block read (0xFF + CH1 block + CH2 block) |
| `FPGA53_SWAP_ORDER=1` | read the 0x05 window before 0x04 |

All of these were tried against the CH2 question; none enabled it — the
negative map is in issue #18.
