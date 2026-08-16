# Bench journal: the 2C53T scope on the 23t port

Device: 2C53T board V1.4, stock firmware V1.2.0. Code: this repository, branch
`2c53t-port`. Variant builds: `dist/variants/`.

Translated from the working journal, which is kept in Russian outside this
repository. Numbers, dump lines and commit hashes are as recorded on the bench;
negative results are written up as carefully as positive ones, because half the
value of this file is knowing where not to go again.

## CH2 REALLY SOLVED: its relay bank was on the attenuated path (2026-08-16)

**What was wrong.** Stock has two independent relay banks, one per channel, and
one and the same ten-row table drives both under a pin isomorphism
`PC12<->PA15, PE4<->PB11, PE5<->PB10, PE6<->PA10` (upstream `4c8ae0b`, both
range functions traced from the stock image: `0x080088A4` for CH1, `0x08008A58`
for CH2). **Bit 0 is the input PATH select**: HIGH direct, LOW through the
attenuator, about 30x down. Our hand-written pose held CH1 at `0x0D` — the
direct path, and not even a stock code — while CH2's pins sat at `0x02`, the
attenuated one. The two channels were looking at the same probe through paths a
factor of thirty apart.

**Numbers, one run, 50 kHz, 10 us/div** (`2C53T-ch2fe.bin`, CH2 moved to row 0 =
`0x0B`):

| probe | CH1 | CH2 |
|---|---|---|
| on CH2 | 4 counts, 0 edges | **179 counts, 8 edges** |
| on CH1 | **179 counts, 8 edges, T16=1600** | 3 counts, 0 edges |

`T16=1600` is exactly 100 samples per period at 5 MSa/s — the prediction landed
to the unit. This is the independence check that was claimed on 08-14 and never
actually taken. Before the fix, the same point measured 6 counts of envelope.

**PB11 was not released.** Row 0 has bit1 set, so the pin config and arm hold
stayed HIGH. Rows with bit1 clear (2 and 7) were left for later; the pose is
applied after the arm writes, which is the only order in which trying them is
worth anything.

**Input coupling (PD12/PD13) — not the trap upstream fell into.** On their unit
PD12 boots in EXMC alternate function (`GPIOD CRH = BB4BBBBB`), ODR is ignored
and the input is AC-coupled for good. Ours reads `crh0=99119999` — both nibbles
`1`, plain output. The pins were never written either way; now they are driven
HIGH (DC), and the boot value goes into the dump so the question stays answered
by a number.

**Rendering: "one wave then a straight line" was a UI defect, not a capture
one.** The raw window strip (`S2`, 64 samples across the whole window) showed a
clean square end to end while the screen drew a flat line. The frame holds 831
samples stretched into 1662 of the 2048 slots and the rest is the last sample
repeated by the read; the renderer clamped the index to the end of the buffer
(2046), and the trigger search scanned the whole buffer and wrapped its offset
with a mask, so an edge to the left of the requested column produced a start
near 2000. Both fixes are in terms of the capture: search up to
`FPGA53_FRAME_VALID`, offset clamped into `[0, valid - visible]`. Confirmed on
the device. Commits `f366cbb` and `70c522e`.

**Instruments left in the firmware:**
1. `C1/C2` in the dump — trimmed window, envelope across frames (cleared by the
   read), edges, period x16, glitch counter. For both channels.
2. `S1/S2` — 64 raw window samples at a step of 12. This is what distinguishes
   "eight periods in the window" from "two and then a rail".
3. `FE` — `crh0`/`crh`, CH2's bank code, and the actual levels of all ten
   frontend pins read back from IDR.

**Traps that cost iterations:**
1. **The envelope as a detector.** The spread inside a single window (166 us) on
   a 50 Hz input is blind by construction: a live channel looks like a dead one.
   The 08-13 conclusion "GPIO candidates exhausted" is withdrawn for exactly
   this reason — the frequency of that run was never written down, and the
   detector was this one.
2. **The timebase step is restored from settings at boot** (`src/ui.c`), and the
   knob is an encoder. After every self-flash the device comes up on the saved
   step, and the physical knob position says nothing about it: check the label
   on screen or the `TB` line in the dump.
3. **The glitch threshold of 30 samples is tied to 50 kHz at 5 MSa/s.** At
   1.25 MSa/s a half-period is 12.5 samples and every honest edge counts as a
   glitch. Read `G` only at the timebase its threshold was computed for.

## THE RELAY LADDER IS THE VOLTS/DIV LADDER (2026-08-16, evening)

**Method.** Sweep CH2's bank rows (`make release-2c53t-rsweep`, table `RSW`) at
four input amplitudes: 20 mV, 100 mV, 300 mV, 1 V. A row's gain is taken as the
SLOPE between two unclipped points, so the constant contribution of noise (five
counts at the coarse rows, up to fifteen at the sensitive ones) drops out
instead of inflating the small spans. Check on the method: row 4 came out at
7.4 mV/count from the 100/300 pair and 7.69 from the 300/1000 pair,
independently.

| row | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| mV/count | 0.4 | 0.80 | 1.98 | 4.0 | 7.5 | 19.4 | 35.0 | ~117 | ~233 | ~350 |
| 1-2-5 | 0.4 | 0.8 | 2 | 4 | 8 | 20 | 40 | 80 | 200 | 400 |

**At 25 counts per division that is exactly ten 1-2-5 steps, 10 mV/div through
10 V/div** — stock's ladder. Our nine steps (from 20 mV up) map onto rows 1..9
by adding one; row 0 is stock's 10 mV/div step, which our UI does not carry. The
binding came out as an identity, with no fitting and no remainder left for
software.

**Bench check, one input (1 V, x1 probe), three steps:**

| step | row | predicted | span | on screen |
|---|---|---|---|---|
| 200 mV/div | 4 | — | 134 | 1.072 V |
| 500 mV/div | 5 | 52 | 55 | 1.100 V |
| 2 V/div | 7 | 13.4 | 14 | ~1.12 V |

Ratios 2.44 (ideal 2.5) and 3.93 (ideal 4.0): three rows a factor of ten apart
agree within 3% and report the same voltage.

**Releasing PB11 is safe — measured.** Rows 2 and 7 clear bit1, taking PB11 LOW
while config and arm hold it HIGH. At 2 V/div (row 7) the dump showed
`ch2[a15,b11,b10,a10]=0011`, frames kept coming, and the window carried a clean
square. Upstream's claim now holds on our hardware too.

**The UI's vertical was rewritten:** volts are computed as "volts/div / 25" from
the zero at `FPGA53_ZERO_COUNT=40`, replacing the inherited 2C23T expression
`delta*754/range_code` referred to 128. That expression was arithmetic on a
number nobody had measured — it is how a 300 mV input read 68 V peak-to-peak.
Commit `77d16fb`.

**Open:** (1) the absolute reading is 7-10% above the generator's dial — the
measured slopes 19.4 and 7.5 against the ideal 20 and 8 account for 3-6%, the
rest is unknown; a reference is needed and one is at hand, the device's own
multimeter on DC. (2) Rows 8-9 rest on differences of three and two counts and
need volts at the input. (3) One zero for both channels; CH1's baseline has to
be re-measured now that it is on the ladder too. (4) `G` on CH2 while clipping
(4233/6145 at index 08) — revisit on an unclipped signal.

**Trap:** the first sweep pass after power-up is not trustworthy — it recorded
20 counts for a row the same dump showed at 63. The sweep has looped and
numbered its passes ever since; take `pass>=2`.

## BREAKTHROUGH (2026-08-12, evening): config entry solved with the 2C23T V0.4 sequence

**`2C53T-v04-config.bin` configures the FPGA from a cold boot. Reproduced 4/4.**
Telemetry: V=0120681B (IDCODE ok), B=00039020 (NV baseline before), A=0003F460
(status after — "design loaded", changes deterministically). After configuration
the ADC is alive: baseline ~0x46-0x4C, responds to the generator (Vpp 0.377 ->
1.885 V). Stock takes no part in the chain at all. The 0x80 marker bit floats
until the first SSPI read — mask it.

**Open facts around it:**

1. **The five stock writes need a pause**: immediately after 0x3A the reply to
   status read `0x03` is zeros; with a 600 ms pause (as stock does per
   maksidze's capture) the reply is meaningful.
2. **The reply to `0x03` is not a status**: the trailing bytes (2-4) change
   between runs and match the range of the live ADC baseline, so they look like
   live samples. The meaningful byte is [1]: stock `01`, ours always `00`.
   Hypothesis: an armed/running flag.
3. **The capture engine fills exactly one buffer and stalls** (R1 C0 W0, after
   which only forced reads): precisely the "captured one buffer then stopped"
   symptom from upstream's R3 netlist analysis. No re-arm happens. Retrying the
   five writes (10 x 200 ms) and a 0x83 preamble before the frame do NOT re-arm
   it.
4. **The SRAM configuration survives 10-20 s without power** (capacitors); an
   honest cold boot means USB disconnected for 30-60 s or more (the battery can
   stay in, though removing it is faster). Honesty detector: B=00039020 (the
   GW1N-UV2 NV status) means power really went away.
5. USB power holds the board up: pulling the battery with USB connected does NOT
   reset the FPGA, and flashing over IAP always leaves the FPGA warm.

**Builds of this stage** (all: V0.4 config at boot, V/B/A telemetry plus
ID/COLD/CFG/Q/ENG verdicts on screen against references):

1. `2C53T-v04-config.bin` — plain config entry (commit 882eacd). POSITIVE.
2. `2C53T-v04-sendcfg.bin` — plus the five writes immediately. Q all zeros,
   engine dead.
3. `2C53T-v04-sendcfg-d600.bin` — plus a 600 ms pause. Q came alive (not zeros),
   engine still dead.
4. `2C53T-v04-kick.bin` — plus write retries until stock's Q (10 x 200 ms), plus
   a 0x83 preamble per frame, plus the on-screen verdicts. One buffer at boot,
   dead afterwards. Q[1]=00.

**Hunting the arm bit (all negative, 2026-08-12 late evening):**

1. `2C53T-v04-sweep01.bin` — automatic sweep of register `0x01` (all 256 values,
   PC0 pulse detection, at least two full laps) — no hit.
2. `2C53T-v04-runpins.bin` — PD3 HIGH from bring-up, PD2/PC4 HIGH after
   configuration (PD2 is "asserted on scope-mode entry", the top candidate from
   upstream's unmapped_mcu_fpga_pin_candidates.md, tested here for the first
   time against a live configuration) — the engine is just as dead (rare forced
   frames).

**Conclusion of the stage:** the firmware-cheap candidates are exhausted. The
question for upstream (netlist): which SPI register bit feeds the CEA/run path
(R3 traced one bit from SI to capture-enable — the register and bit identity is
what is needed), and whether IOB7B is PC6 or PC11.

## What works (positive results)

1. **The 23t port on 53t hardware**: button matrix 15/15 (scan taken from the
   53t `button_scan.c`), PC9 power hold, PB8 backlight, battery on PB1/ch9,
   SCOPE splash screen. The UI is fully operable.
2. **Warm handoff**: the FPGA's SRAM configuration survives reflashing the MCU
   as long as board power is not interrupted. Entering IAP without dropping
   power: **MENU + pinhole reset**. Works stock->port and port->port. Any power
   loss (including unplugging the USB cable) loses the configuration; recovery
   is stock -> scope mode -> handoff.
3. **Transport**: hardware SPI3 mode 3, master, soft CS on PB6 (idle HIGH),
   PC6 HIGH, PB11 HIGH, PC0 as data-ready (input). A read is CS low, opcode
   (`0x04` CH1 / `0x05` CH2), two more status bytes, 1023 samples, CS high —
   1026 bytes per window. ADC offset -28.
4. **The key to capture is DAC1**: the trigger level is DAC1 (PA4,
   `DHR12R1 @ 0x40007408`); an MCU reset zeroes it, leaving the comparator
   without a reference and the frames empty (zeros). Restoring DAC1 to
   mid-scale gives a **live CH1 trace** (external generator, square wave, Vpp
   computed).
5. **PC0 handshake**: in the live state PC0 pulses (~20% duty) and every read
   follows real readiness. The `0x80` byte in MISO marks the first CS window
   after readiness — it is not a property of the opcode.
6. **Bus writes are safe** for a live FPGA (preambles every frame, config writes
   at boot — the handshake does not break). The early FPGA deaths came from
   power loss, not from writes.

## Did not work: CH2 (every hypothesis tested, 2026-08-12/13)

Symptom at the time: both windows carry CH1 (`0x04` the direct path, `0x05` an
attenuated copy with a different baseline, i.e. the second ADC). A signal on CH2
appears nowhere, even when stock had been showing CH2 immediately before the
handoff.

> Resolved 2026-08-16: the CH2 bank was sitting on the attenuated row of the
> stock relay table while CH1 was on the direct path. See the top of this file.

1. Relays/attenuators: 16 patterns of PC12/PE4/PE5/PE6 — no (PC12 is an audible
   relay, the same click stock makes going 2V -> 5V).
2. Gain keys: 16 patterns of PA15/PA10/PB9/PA6 — no. (PB9 later turned out to be
   the board's buzzer and PA6 the TMR13 reference, so this bank was never
   actually swept — upstream `d35f417`.)
3. Window read order: swapping does not change the distribution — the opcodes
   select sources and the order does not matter.
4. A `0x80|x` preamble before the frame: fixed `0x83` — no; an automatic sweep
   of all 128 values (`A80.`-`AFF.`) — no.
5. The five stock config writes (`01 08, 02 03, 06 00, 07 00, 08 AD`): accepted,
   they change behaviour (PC0 goes permanently high, W=0), and status read
   `0x03` returns `80 00 00 00 00` where stock in capture got `00 01 42 2E 2E` —
   CH2 still does not appear.
6. Register `0x02` is the **capture-mode selector**, not a channel mask:
   `02 00/01/02/04...` kill the capture (PC0 dies, only emergency frames),
   `02 03` keeps the engine alive. The value sweep was aborted because the
   emergency reads are slow.
7. Mode `02 04` ("dual" from the old RE) plus a block read (0xFF + 1024+1024):
   the FPGA answers with zeros (not FF) — the bus is alive, the capture is not.

## Multimeter (2026-08-13, night): stage 1 passed

1. **The DMM transport is alive**: `dmm53.c` — USART2 PA2/PA3 at 9600, wake
   preamble (`05 08/09/probe/14`), poll `00 09` at 4 Hz gives a steady stream of
   12-byte `5A A5` data frames. There are no echo frames (`AA 55`) — the SoC
   streams without them.
2. **The port's PCLK1 is 96 MHz** (auto-baud, BRR=0x2710). Use this when setting
   any baud or timer values in the port.
3. **The device's SPI flash is a clone**: Zbit ZB25VQ128 (JEDEC90 = `5E17`), not
   a Winbond; `w25q.c` now accepts any manufacturer with density 15-17.
4. **A direct Mac-to-device channel works**: the port brings up USB MSC ("W25Q
   RAW FLASH", FAT12 with a 4096-byte sector); a long SAVE press writes
   `img_NN.bmp` to the root of the volume. Retrieval: `diskutil unmount disk4 &&
   diskutil mount disk4` (mandatory — otherwise macOS shows a stale FAT cache),
   then copy the BMP and convert. Photographs of the screen are no longer
   needed.
5. Meter debug overlay: three lines under the panel, refreshed every 500 ms:
   `B<baud><?|!> W<wake> T<tx> R<rx> D<data> E<echo> X<err>` /
   `F <12 frame bytes>` / `EC<echo> BRR<hex> P<MHz> ID<flash> C<MB>`.

## Autonomous development loop (2026-08-13): flashing and screenshots hands-free

**The port's USB self-update works on the 53t.** A full cycle with no IAP and no
user involvement beyond pressing buttons on the device:

1. Build, then `cp <bin> "/Volumes/NO NAME/F2C23T-UPDATE.BIN"`, sync, unmount.
2. The device scans the FAT after about a second of idle, stages to 0x08040000,
   shows "flashing firmware" and reboots into the new image. Verified twice (the
   #2 marker in the U line reached the screen).
3. Requirements: the name contains `F2C23T`, the extension is .BIN, the size is
   at least 8 KB and at most 224 KB (stock's 751 KB does not fit — for stock it
   is still IAP via MENU + pinhole).
4. Traps: volume 1 is only 2 MB (factory assets, 150-450 KB free) — delete old
   screenshots before copying; delete AppleDouble `._*` files (harmless to the
   matcher, which filters on size >= 8K and .BIN, but they waste space); after
   EVERY change from either side, `diskutil unmount disk4 && diskutil mount
   disk4` (the two owners' FAT caches do not synchronise); and the device's
   reboot is easy to miss by polling — trust the text on the screen.

**Screenshots**: long SAVE press on the device, `img_NN.bmp` in the root of
volume 1, remount, copy, convert, read. The BMP is written with a dark palette
regardless of the screen theme (a quirk, not fixed). The MSC write counters are
exposed in the U line (`W<chunks> X<fail>@<site> L<flush-ok> H<find-hits>`).

## Multimeter stages 3-4 (2026-08-13, ~02:30, SUSPENDED)

1. The decoder is ported: `src/meter_data.c/h` — a copy of upstream's (BCD
   decoding of seven-segment nibbles, OL/CONT/blank, sign, decimal point, units)
   with shims instead of string.h and a HW_TARGET_2C53T guard. Wired into dmm53
   (`dmm_value_text` -> display_str and so on), with the port's UI modes (0-12)
   mapped to submodes (0-9) in `dmm53_submode()`.
2. SoC mode commands added (`dmm53_send_mode_sequence`, a copy of upstream's
   fpga_send_meter_mode_sequence): RESET -> the per-submode set -> START/probe.
   Step 4 of the wake machine.
3. **Decoder check**: a 300k resistor reads about 300k for the first fraction of
   a second (the decode works), then a relay clicks and "90 Ohm"/OL alternate —
   the SoC's autorange is hunting.
4. Known gaps: the 300k band is not covered by the range table (a raw frame from
   the screen at the "90 Ohm" moment is needed); the frontend relays do not
   switch by mode (the DCV pose is left in place); AUTO DETECT has no mode
   command.

## Upstream's overnight answers (2026-08-13, issue #18) — ARM SOLVED

1. **The maintainer reproduced our bit-banged config entry** (Build B:
   transplanted loader -> 0003F460 DONE_FINAL SET; their hardware SPI at /256
   still refuses). The carrying factor is the GPIO bit-bang (confound: their
   hardware path sends 0x05 ERASE_SRAM, the bit-bang does not).
2. **They achieved a full cold bring-up**: the five writes arm the engine if
   **PB11 is held HIGH BEFORE the writes** plus PC6 HIGH (a three-way AND from
   the netlist). Builds on their main: guest-configB / guest-configB-arm /
   guest-coldtrace; document: docs/build_a_config_entry_test.md.
3. **Our puzzle**: we held PB11 HIGH the whole time and still did not arm. Their
   arm mechanics differ from our SEND_CFG block in some detail (write clock and
   framing? pauses? order?). First step of the scope track: pull upstream, read
   their arm code, diff against our kick variant, port it into v04.
4. maksidze is doing a three-way logic-analyser comparison (stock /64 good,
   bit-bang good, their hardware SPI bad); the first test is hardware SPI
   without 0x05.

## Scope track (2026-08-13, day): COLD BOOT -> LIVE CAPTURE ON THE PORT

**`2C53T-v04-arm256-pc0low.bin`: cold boot -> ID:OK COLD CFG:OK ENG:RUN,
R166/C1/F166 — reads follow PC0 readiness, the trace updates smoothly.** Two
details were missing against upstream's arm mechanics, not one:

1. **The arm writes and the 0x03 status read go at /256 (~470 kHz)**, not at the
   working /8: their guest-coldtrace switches BR around the five writes (2 ms
   gaps, 600 ms pause after 0x3A). Ported into the FPGA53_SEND_CFG block (BR=7
   going in, the working divider coming back). With only that change
   (`2C53T-v04-arm256.bin`) the first buffer arrived (R2 C1 ENG:RUN in the
   photo) and then ENG:DEAD — but the min-max DID change between frames and
   r0=0x80 (the fresh-buffer marker), so the engine was re-capturing and only
   the rare forced read was catching it.
2. **PC0's polarity is inverted**: upstream's working cold rig reads readiness
   as LOW with a pull-up (undriven = HIGH = not ready); we had HIGH and
   floating. New flag `FPGA53_PC0_READY_LOW=1` (polarity plus the pull-up in
   fpga_init_once). The warm-handoff behaviour (HIGH) stays the default — do not
   break what is proven.
3. Q stays BAD: the 0x03 reply is `00 00 4B 44 44` (byte[1]=00 against stock's
   01) with a working engine, so the "armed=01" signature is NOT necessary and
   bytes 2-4 are live samples. Worth mentioning in the next #18 report.
4. Recipe for the winning build: the release-2c53t flags plus
   `FPGA53_V04_CONFIG=1 FPGA53_SEND_CFG=1 FPGA53_SEND_CFG_DELAY_MS=600
   FPGA53_PC0_READY_LOW=1`.
5. Bench trap: a BMP screenshot (150K) cannot be written while our 223K BIN sits
   on the volume (the device reports an error = no space; it deletes the BIN
   itself after flashing). To make room, the factory `System file/81,83,85.jpg`
   were deleted — they exist byte for byte inside `w25q128_myunit.bin` (hashes
   compared) and are restorable.
6. A warm reflash (USB self-update without dropping power) leaves the FPGA
   configured and SSPI silent, so v04_id=0 and overlay lines 3-5 are hidden (by
   design, not a bug); the trace is alive throughout.

## After the success: CH2 / timebase / bench (2026-08-13, day)

1. **TMR13 test for CH2 — NEGATIVE as a single measure**:
   `2C53T-coldtrace-tmr13.bin` (winning base plus FPGA53_TMR13_REF), generator on
   CH2, and the CH2 window stayed flat (b46-49) — the signal does not reach the
   second ADC. The engine meanwhile was flying (R9535/C1). Conclusion at the
   time: CH2 is an input ROUTING problem and the trigger reference is secondary.
2. **Favourite hypothesis for CH2** (later confirmed as the right area, wrong
   mechanism): with SCOPE_ANALOG_CONFIG=0 the frontend relay bank FLOATS — our
   own conclusion, "CH1 conducts by accident and CH2 is open" — while upstream's
   working coldtrace poses the whole bank, including **PB10, which had never
   appeared in any of our sweeps**. New flag `FPGA53_FE_SCOPE_POSE=1`: their
   case-7 pose plus PC12 HIGH at boot.
3. **Jumping lines, no square shape, nothing at 500 ms - 10 s — EXPLAINED, not
   an arm bug**: the FPGA timebase is not programmed (fpga_write_timing is a
   stub), sampling is always full rate, and a 1023-sample window is tiny, so a
   kilohertz square is almost always "a flat level jumping between top and
   bottom". The port's slow timebases go through
   fpga_capture_read_slow_point (a stub), hence empty.
4. **Bench: the volume was cleared for screenshots** (~1.8 MB free): all 157 JPGs
   from `System file/` that matched the dump byte for byte were deleted (the
   reference is w25q128_myunit.bin); `141.jpg` DIFFERED from the dump and the
   live copy was saved before deletion. Restoring the stock assets is an mcopy
   from the dump back onto the volume.
5. Bench idea (later implemented): debug telemetry straight over USB instead of
   screenshots — the device already scans the FAT for F2C23T-UPDATE.BIN, so let
   the host drop a DBGREQ file and have the firmware write DBG.TXT with the diag
   structure. No flash wear while idle and no buttons to press.

## The PC0 model revised, and paced reads (2026-08-13, evening)

1. **The frontend relay pose WORKS**: with it an external generator draws a
   readable shape — CH1's input path and coupling are fine.
2. **PC0 is not "data ready" in one polarity, it is an engine-state
   indicator**: with no signal the engine free-runs and pulls PC0 LOW (the
   LOW-readiness reading catches it, frames are frequent); with a signal it is a
   triggered capture, PC0 never asserts, and frames only come from forced reads
   (about one every ten seconds) — but every forced read is a fresh triggered
   buffer. The old HIGH polarity worked on the warm bench with a generator for
   the same reason (triggered captures hold PC0 HIGH).
3. **The fix is stock's cadence**: stock does not wait on PC0 at all, it reads
   the 04/05 pair every ~29 ms unconditionally, and the read itself re-arms the
   engine. New flag `FPGA53_READ_PACED=1` (fpga_capture_ready always 1, PC0
   demoted to diagnostics).
4. **The USB debug channel is ready**: drop an EMPTY file named `DBGREQ` (no
   extension) on the volume and the device's idle scan (about a second) deletes
   it and writes `DBG.TXT` — the overlay telemetry plus every port's IDR plus
   SPI3 CTRL1. Rewritten in place, no cluster leak. Code: `src/dbgdump.c`, hook
   in `usb_msc.c raw_fat_service_dbgreq`.
5. Q:BAD on an honest cold boot, confirmed again; it does not bother the engine.

## CH2: GPIO candidates EXHAUSTED (2026-08-13, evening)

> **Withdrawn 2026-08-16.** The detector used here was the spread inside a
> single window, which is blind to any input slower than the window; the run's
> input frequency was never written down. Two of the four pins in bank B were
> also wrong: PB9 is the board's buzzer and PA6 is the TMR13 reference, so CH2's
> real bank was never swept at all. See the CH2 section at the top.

1. All negative with a live paced capture and the generator on CH2 (observed
   through DBGREQ dumps, both windows staying at their baselines): bank A
   PC12/PE4/PE5/PE6 (16 patterns), bank B PA15/PA10/PB9/**PB10** (16, PB10 for
   the first time), the TMR13 PWM reference (PA6), the whole stock scope pose,
   and the selector **PC2=H/PC1=L** plus **PC11=L** (the meter MUX; the pose
   confirmed by the IDR dump C=B7FC).
2. Conclusion at the time: CH2 routing is switched INSIDE the FPGA by runtime
   registers over SPI3 (candidates 0x06/0x07/0x01 from the five writes). Next
   experiment: sweep the values of 0x06/0x07/0x01 on a live armed engine with a
   CH2 spread autodetect.
3. For #18: upstream's "live capture on both channels" may be our own artefact
   (the second ADC carries an attenuated copy of CH1, so with the probe on CH1
   "both channels respond"). Our dumps separate the two cleanly: probe on CH2,
   both windows flat. Worth checking on their side.
4. Builds of the day: `2C53T-paced-dbg.bin` (paced plus the DBGREQ channel),
   `2C53T-paced2-mux.bin` (plus the PC1/PC2/PC11 pose and warm overlay lines).
   Flag recipe: the winning base plus FPGA53_READ_PACED=1 plus
   FPGA53_FE_SCOPE_POSE=1 plus FPGA53_TMR13_REF=1.

## CH2 "confirmed" plus meter, pose, and a timebase measurement (2026-08-14)

> **THE CH2 CLAIM WAS WITHDRAWN ON 2026-08-16.** The observation "the blue line
> jumps" was made by eye; no numbers on CH2's response were taken that day. The
> 2026-08-16 measurement on the same bench gave 6 counts of envelope against
> CH1's 189 — the channel was not working. The real cause and the real numbers
> are in the CH2 section at the top of this file. The rest of this section (the
> meter, the frontend pose, the bench traps) still stands.

**CH2 solved — channel independence demonstrated live.** User procedure: battery
out, USB in (an honest cold boot), boot into scope, into the meter (select ->
diode, move through AC/DC/resistance, hold move -> auto), hold select -> scope.
**Probe on CH2: the blue line jumps, the yellow one rests. Probe on CH1: the
yellow one jumps, the blue rests.** The "both ADCs carry a copy of CH1"
hypothesis is dead. The dumps confirm it numerically: with a signal on CH1,
`ch1` moved from its `4D-50` baseline to `AD-B1` while `ch2` stayed at `45-48`.
Build: `2C53T-paced2-mux` plus the meter fixes.

**Meter: the stage is unblocked and autorange no longer hunts for nothing.**
Taken from upstream's merged PR #13: (1) **the fifth digit** — on `frame[2].3`
stock adds 10000 to the four-digit raw value (`FUN_08036AC0` @ 0x08036BFC);
neither we nor upstream's main had this, so everything above 9999 counts decoded
into garbage. Applied for DCV only; in the other submodes the bit is merely
reported as `E0/E1` in the debug line. **Verified on the device: 12 V reads
correctly.** (2) `dmm_set_mode()` no longer restarts the preamble and mode
sequence when the mode is already active (the UI called it on every AUTO press),
so the SoC gets its ~2.5 s to settle. Commit `8dfc2f4`.

**Frontend pose: a hole found and closed.** The scope pose and TMR13 were applied
only inside the config path, i.e. once at boot; the meter sets its own pose and
`dmm_pause()` clears only PC11, so a trip through the meter left PE4=1/PE5=0
(scope wants 0/1), PA15=1/PA10=1 (wants 0/0) and PA6 as a plain GPIO instead of
the TMR13 PWM. Both blocks moved into functions plus
`fpga53_scope_pose_reapply()` on scope-mode entry. Before/after numbers in commit
`487f2a7`.

**Timebase measured (external generator, 50 kHz square, 3 V, 50%).**

> Superseded 2026-08-15: this reading was an estimate by eye and it was wrong.
> The screen was showing only 300 of 2048 frame entries, about 15% of the
> window. The measured numbers are 5.00 MSa/s and a 205 us window.

1.5-2 periods on screen -> a 1023-sample window of about 33 us -> an effective
sample rate of about 31 MSa/s. At 110 Hz the window fits inside one half of the
square: a spread of 4 counts with the level jumping between frames, which is
what the "jumping line" is — not a shape. Vertical on the case-7 pose: about
30 mV/count, full scale about 7.7 V (3 V gave 99 counts).

**New defects for the pile (they do not block capture but they explain
oddities):**
1. `r1=00` in every dump where stock has `01` — our candidate for an "armed"
   flag. It never once came up.
2. **A spurious top bit in SPI3 readbacks**: `V=8120681B` instead of
   `0120681B`, `A=8003F460` instead of `0003F460`, and `r0=0x80` on screen. A
   one-bit shift on some reads.
3. **The signal generator does not work on 2C53T at all**: `siggen_configure`
   writes its DDS word through `fpga_write_timing`, which is a stub on this
   board. The hardware is a dual 12-bit DAC. Implementing the 2C53T path would
   open up the generator, Bode plots, and a signal source for the bench.

**Bench traps that cost iterations (more important than they look):**
1. **`make release-2c53t` with no flags is "FPGA stubbed (no scope trace)"** —
   it says so in the Makefile. We flashed it once and lost the signal. (The
   size marker that used to identify it stopped working on 2026-08-16, when the
   bitstream moved out of the image.)
2. **`F2C23T-UPDATE.BIN` disappears from the volume even when the update is
   REFUSED** — the file being gone proves nothing. The sign of a firmware that
   actually applied: reset counters in the dump (`calls init`, `R`) and a
   recreated filesystem on the volume.
3. **A warm reboot after a self-flash does not configure the FPGA** — `V/B/A`
   are zeros in the dump and there is no trace, although levels still read.
   (Fixed 2026-08-15 with the warm token; an honest cold boot is only needed
   when the bitstream changes or power really goes away.)
4. **DBGREQ needs a pause**: write the file and read `dbg.txt` in separate
   passes (the idle scan takes about a second), otherwise you get the previous
   dump. Copy with `COPYFILE_DISABLE=1` or macOS litters the volume with `._`
   files.

**Upstream over the same day:** PR #13 (meter) merged into main; the timebase
question reopened — `BSRAM_0/3` (CH1/CH2) are written with the raw PLL clock
while `BSRAM_1/2` use a gated GB40 whose enable cone contains the SPI receiver,
making it a candidate for the roll buffer; `0x05 ERASE_SRAM` ruled out as the
cause of the hardware-SPI refusal; a per-device factory calibration found at
0x08006000, with `make guest-caldump` — **a CRC32 from a second unit is needed,
i.e. from ours**.

## Timebase: MCU pacing plus a falsification instrument (2026-08-14, night)

**The pacing idea.** The engine free-runs and every read hands back the freshest
window; for anything slower than about 30 kHz that window is a single
instantaneous level. So the MCU owns the timebase: take one point (the mean of
the first few samples of a channel, `FPGA53_SLOW_POINT_SAMPLES`) on a TMR1 tick
and you get an honest slow timebase without depending on FPGA registers
(`fpga_write_timing` was and remained a stub).

The port's roll infrastructure was already there (timer, ring buffer, soft
trigger on roll points, rendering) and idle only because
`fpga_capture_read_slow_point()` returned 0 on 2C53T, which is why every slow
timebase showed nothing. Implemented, plus:

1. **The roll floor was lowered** from index 21 (500 ms/div) to 15 (5 ms/div),
   in a `HW_TARGET_2C53T` branch that leaves HW40 alone.
2. **The point interval is computed in ticks, not whole milliseconds** (before:
   any floor under 1 ms became 3.3 ms/div). A TMR1 tick is 100 kHz, so the step
   is 10 us. The table was verified on the host: 5 ms/div -> 200 us per point
   (5 kSa/s), 50 ms/div -> 2 ms per point, 10 s/div -> 400 ms per point (40000
   ticks, fits in 16 bits).
3. **SPI3 bus arbitration** (`fpga53_bus_busy`): the sampler lives in the TMR1
   IRQ while full window reads and configuration live in main; skipped points
   are counted (`busy=` in the dump). The flash is on SPI2 and the meter on
   USART2, so they do not conflict.
4. **A scale trap, noted:** `scope_timebase_unit_ns[]` is a TENTH of ns/div
   (index 18 is 5 000 000 and the label says "50MS"). The first version of the
   computation was off by ten; caught before the bench.

**110 Hz at 5 ms/div is 45 points per period** — the point of the whole
exercise. The gap between the fast branch and 60 ms across the screen remains;
it closes either with a hardware timebase or with ETS.

**Open bench question 1: the timer clock.** The port does not program the PLL and
`SCOPE_SLOW_TIMER_CLK_HZ` = 72 MHz is an ASSUMPTION (the meter's auto-baud
sweeps 36-120 MHz). The timebase scales linearly with this constant. One-pass
calibration: feed a known frequency, read the period off the roll trace,
multiply the constant by (shown/true). `DBG.TXT` prints the `psc=`/`pr=` actually
programmed.

**Open bench question 2: does the engine survive a shortened read.** A point
reads three status bytes plus eight samples and drops CS instead of draining all
1023 bytes. If that stalls the engine, rebuild with
`-DFPGA53_SLOW_POINT_FULL=1`, which drains the whole window — slower, but byte
for byte like a normal read.

**The timebase falsification instrument (a shape metric).** Sample-rate
experiments used to end in "did it stretch or not, by eye". Now the CH1 window
is measured by a number: mid-level crossings going up (hysteresis at +-1/8 of
the span) and the mean period between them in samples x16 — the `e=`/`T16=`
fields in `DBG.TXT`. Verified on the host against synthetic data: period 620 ->
616.0 (-0.65%), 310 -> 309.6, 155 -> 155.1, and 77.5/38.75/10 exactly; a flat
line and a signal below the spread threshold (8 counts) give 0 rather than noise.

**The build trap is closed in the Makefile.** There are now
`release-2c53t-scope` and `release-2c53t-tsweep` targets; both wipe
`build/2c53t` first, because object files do not remember their `-D` flags and
without the clean the link is a mixture of two firmwares.

## TIMEBASE: measured on the bench (2026-08-15) — three numbers and one closed question

**1. The timer clock is 240 MHz, not 72.** Measured without eyes: two dumps a
known wall-clock interval apart, and the difference of the 32-bit point counter.
At `psc=719 pr=19` (nominally 200 us per point under the old 72 MHz assumption)
the sampler produced **17 008 points/s = 58.8 us**, i.e. 3.40x faster.
240/72 = 3.33 — it fits, and 240 MHz is the AT32F403A's ceiling. The port does
not touch the PLL, so that is what the factory bootloader leaves.
`SCOPE_SLOW_TIMER_CLK_HZ` = 240e6. **The point counter had to become 32-bit**: a
16-bit one wraps in seconds and the first measurement came out ambiguous.

**2. The FPGA window refreshes ONLY after it has been read out in full.** The
decisive comparison, both lines from `DBG.TXT` on the same device with a live
signal:

| point mode | spread of points over a block of 256 |
|---|---|
| short read (3 status bytes + 8 samples) | `p=92-92` — a constant |
| full window drain (1026 bytes per channel) | `p=00-94` — the whole range |

A short read hands back a frozen buffer: poll it as often as you like (and we
polled at 17 kHz), it is the same level. That is where the "jumping level" and
"period far wider than the screen" symptoms came from — we were seeing the
window refresh rate, not the signal, with the square aliased onto it. The
hypothesis "the engine re-arms on the fact of being read out" is confirmed.

**3. A point costs 1.6 ms** (`cost=531` ticks of 3 us, both channels, 2052
bytes, SPI at ~12 MHz — consistent with the arithmetic). Hence the ceiling of
the slow timebase: about 625 points/s in theory, 250 points/s with responsiveness
taken into account.

**Consequence for the roll floor: 100 ms/div (index 19).** At 4 ms per point
that is 40% CPU. Verified the expensive way: at 50 ms/div (2 ms, 80%) and at
500 ms/div before the clock was calibrated (a real 6 ms instead of 20) the device
stopped servicing the USB volume — the interface stays up while transfers hang
for minutes. The original 5 ms/div was a fantasy; the hardware does not give
that.

**110 Hz still did not become a shape** — at 250 points/s that is 2.3 points per
period. Getting a shape needs at least 1 kSa/s, i.e. a cheaper read: (a) SPI
from 12 to 24 MHz, (b) DMA instead of byte polling, which also removes the USB
starvation.

**A separate bench finding: the telemetry channel does not survive roll.**
DBGREQ -> DBG.TXT needs an unmount and an idle moment, and with the sampler
running the volume stops mounting at all. Workaround: switch the device to a fast
step before taking a dump (roll stops and the counters survive — they only reset
when roll starts). The real fix is a live stream over MSC.

## Fast timebase started working: the whole window is shown (2026-08-15, night)

**We had been showing 15% of the capture all along.** `scope_visible_sample_count()`
computed the visible points from `SCOPE_FAST_HW_SAMPLE_NS = 20` (a 2C23T legacy,
50 MSa/s) and capped the result at 300 samples. Our buffer entry is **100 ns**
(5.00 MSa/s with the 2x stretch) and the frame holds 2046 entries, so about 30 us
out of 205 captured reached the screen and every fast step looked identical. This
also explains the 08-14 measurement error: "1.5-2 periods of 50 kHz visible" was
about the crop, not the window.

Fix: on 2C53T the visible sample count is computed from the requested time per
division and the real sample interval, capped by the frame. **The timebase knob
started working at the fast end for the first time:** 1 us/div -> 12 us on
screen, 5 -> 60 us, 10 -> 120 us, and from 20 us/div it hits the window size and
shows all 205 us. Confirmed by the user: the picture changes from step to step.

Build `2C53T-fullwindow.bin`, dump line `WIN e=10 T16=1600` — exactly 100 samples
per 50 kHz period, so 5.00 MSa/s is confirmed a second time.

## The timebase knob: two refuted explanations and a lesson about fragile estimates (2026-08-16)

After binding `01 <idx>` to the knob, index `0x11` (10 ms/div) read sometimes as
250 samples per period and sometimes as 202-212. I built two explanations in a
row, both wrong, and spent half a dozen bench iterations on them.

**Hypothesis 1: the command steps rather than sets.** It rested on two points:
`0F->11` in one jump gave 202, `12->11` as a neighbouring step gave 250. Stock
never jumps either (it holds `idx` and steps to `idx+1`). I implemented
single-stepping plus a descent to zero at boot to have a known origin. **It did
not help.**

**Hypothesis 2: the intermediate steps need a longer dwell.** I gave each step
stock's `LUT[idx]+0x32` ms instead of 1 ms. It moved 3280 -> 3386 and **did not
help.**

**What was actually happening: the period estimate falls apart with few
intervals.** At 20 Hz index `0x11` puts only 4 crossings in the window, so the
period is computed from three intervals and one outlier moves the answer by a
fifth. Re-measured at 50 Hz, where the same window holds 9 crossings: approaching
from below gave `T16=1534`, from above `1524`, in one jump `1524`. **There is no
difference and no path dependence.** The stepping and the descent to zero (about
sixty lines) were deleted; one write plus a full-window dwell remains.

**A real error in the table was caught in passing.** The ratio of neighbouring
indices' periods does not depend on the generator: `T(0x11)/T(0x12) =
95.25/50.00 = 1.905`, while the table assumed 2.008. The anchor is index `0x12`,
the most reliable estimate of the day (17 crossings, 16 intervals): 400.0 us per
sample at 50 Hz against 401.6 us from an independent 2 kHz run, which also says
the generator is honest. So `0x11` is **210 us (4.76 kSa/s)**, not 200.
`fpga53_tb_entry_ns[10MS]` corrected: 100000 -> 105000.

**The lesson, in two lines.** The old rule was "sweep at a frequency that gives
several periods in the window". That is not enough. The rule now: **before
explaining a discrepancy by hardware, look at how many intervals the estimate
was computed from.** Three intervals is not a measurement, it is a hint. The
`WIN` metric prints `e=`; with `e < 6` no conclusion about the rate is valid.

**What stayed in the code from those iterations:** the `TB` line in the dump —
`now/want/ns/calls/skip/sent`. It is what showed, in one pass, that after a
reflash the device comes up on the saved step rather than the one the knob is
pointing at; before it existed I twice mistook correct readings for a failure.

## TIMEBASE WORKS: the knob changes the sample rate (2026-08-16)

The `01 <idx>` command is bound to the UI steps. **Verified on the device with a
50 kHz input** — the predictions were made BEFORE the measurement, using the
`T16` metric in the dump:

| step | idx | f_s | expected `T16` | measured |
|---|---|---|---|---|
| 10 us/div | 08 | 5.00 MSa/s | 1600 | **1600** |
| 100 us/div | 0B | 0.50 | 160 | **160** |
| 200 us/div | 0C | 0.247 | 79 | **80** |

Three points across a 20x range, agreeing to the unit. Before this
`fpga_write_timing` was a stub and every step showed the same window.

**How.** `fpga53_set_timebase(ui_timebase)` in `fpga.c`: a step-to-engine-index
table, the `01 <idx>` write in its own CS window with the same bus discipline as
a read (DMA cancelled, `bus_busy`), and a dwell of one full window at the new
rate (4.1 ms at idx 0C, which is why it is computed rather than constant).
Called from `scope_hw_configure_channels`, i.e. on every step change. The
hardcoded "100 ns per frame entry" in `ui.c` is gone, replaced by
`fpga53_frame_entry_ns()`.

**The slow end was mapped with a 2 kHz input (a second sweep), closing the
gap.** The earlier conclusion "`0F-13` are either >20 MSa/s or a dead capture" is
WRONG: they are the slow end, and the reason they gave two edges or fewer at
50 kHz is **aliasing** — 40 us per sample against a 20 us period is far past
Nyquist.

| idx | period at 2 kHz | f_s | ns/sample |
|---|---|---|---|
| 0B | 250.00 | 0.500 MSa/s | 2 000 |
| 0C | 125.00 | 0.250 | 4 000 |
| 0D | 62.50 | 0.125 | 8 000 |
| 0E | 25.00 | 0.050 | 20 000 |
| 0F | 12.50 | 0.025 | 40 000 |
| 10 | 6.19 | 0.0124 | 80 808 |

`0B` and `0C` match the 50 kHz run. `11-13` gave 2.5-3.2 samples per period —
aliasing again; they need an input around 20 Hz.

**Extended table checked on the device (2 kHz, predictions before the
measurement):**

| step | idx | expected `T16` | measured |
|---|---|---|---|
| 1 ms/div | 0E | 400 | **400** |
| 2 ms/div | 0F | 200 | **199** |

**Coverage as it stands:**
1. **5 us/div through 5 ms/div genuinely works** (indices 07-10) and the label
   on screen matches the time. That is a thousandfold range where in the morning
   there had been a single frequency.
2. **Faster than 5 us/div** clamps at idx 07 (12.5 MSa/s, a 66 us window) and
   the label lies: the engine does not go faster.
3. **10 and 20 ms/div** clamp at idx 10 — they need 120 and 240 ms windows and
   idx 10 gives 67 ms. Indices `11-13` will close them once measured at ~20 Hz.
4. **From 50 ms/div** it is roll, as before.

## THE SAMPLE-RATE DIVIDER FOUND: SPI3 command `01 <idx>` (2026-08-16)

**Open question 2 closed.** The sample rate is set by the two-byte SPI3 command
`01 <idx>`, idx being the timebase index `0x00-0x13`.

**Why we missed it.** On 2026-08-15 we swept the index as a **single** byte in
its own CS window and recorded a no-op. It was a no-op: without the `0x01`
command byte in front, the command is simply incomplete. The form came from
decompiling stock (`button_candidates.c`, case 1): stock sends `01 <idx>`, holds
the index for `LUT[idx]+0x32` ticks, then steps `idx -> idx+1`, clamping at
`0x13`. The dwell table was extracted from the stock image at `0x0804D833`
(image base **0x08007000**): `1` for indices 0-13, then `3, 5, 9, 21, 41, 82`.

**Measurement (50 kHz input, sweep with a 16-frame dwell per step, `make
release-2c53t-tb01`, table `TSW` in `DBG.TXT`):**

| idx | period, samples | f_s | ratio to base |
|---|---|---|---|
| 07 | 250.00 | 12.5 MSa/s | 2.5000 |
| 08 | 100.00 | 5.00 | 1.0000 |
| 09 | 50.00 | 2.50 | 0.5000 |
| 0A | 25.00 | 1.25 | 0.2500 |
| 0B | 10.00 | 0.50 | 0.1000 |
| 0C | 4.94 | 0.247 | 0.0494 |
| 0D, 0E | 2.44 | 0.122 | 0.0244 (metric saturated, 255 edges) |

Ratios of `2.5 / 1 / 0.5 / 0.25 / 0.1 / 0.05` — a 1-2-5 ladder to four digits,
with round periods. Noise does not look like that. Index `00` gives the same as
the base.

**Honesty note:** the ratios do not depend on the generator's frequency, but the
absolute `f_s` values do — they assume exactly 50 kHz. The base of 100.00 samples
agrees with the independent 5.00 MSa/s measurement from 2026-08-15.

**The window metric was fixed in passing.** `fpga53_window_metrics` counted over
the whole window including the dirty head; the spurious crossings from the
glitches inflated the edge count and deflated the period, so the same input gave
`T16=1478` in one dump and `1600` in another. It now trims head and tail like the
renderer and the seam analyser, and the base reads exactly `100.00`. **Before the
fix the sweep still showed the ladder, but with up to 10% error; the conclusion
did not change, the precision did.**

## Stock does NOT drop the head — it brackets the read (2026-08-16, analysis)

We were testing the assumption from the previous section: since stock's trace is
clean, it must be dropping the head of the window. **Refuted by decompilation.**

**Stock draws the first 301 of its 1024 samples, head included.** The
normal-timebase renderer: `raw = adc_buf_ch1[i]` for `i = 0..300`, a VFP
transform, clamp `[0x1C, 0xE4]`. No offset. So if stock's head were dirty it
would be showing it — and it shows 29% of the window starting at sample zero.

**What stock does and we do not do at all**: it brackets every read with three
steps.

1. **`trigger_byte=1`** (case 0, SCOPE_FAST_TIMEBASE, sent twice). No data is
   read. It sends the one-byte timebase index and **gates the read on
   accumulation**: while `acq_count < TIMEBASE_SAMPLE_TABLE[tb_idx]+0x32`, no
   read is allowed.
2. **`trigger_byte=4`** — the buffer read.
3. **`trigger_byte=5`** — the re-arm.

We send none of the three: we read the window whenever the UI wants it.

**Consequence for our defect.** The formulation "the engine inherently needs
26 us after arming" is unsupported. More plausibly: **we read a window whose
beginning the engine has not finished writing**, because we skip the
accumulation gate. In that case the 160-sample skip stands in for a handshake we
never perform, not for hardware physics.

**Directly testable:** implement stock's bracketing, set `FPGA53_HEAD_SKIP=0`,
and watch the `GLITCH` counter — the instrument already exists. If `hit` drops on
its own, the workaround can go and 19% of the window comes back.

**Caveat:** we already swept the one-byte timebase index on 2026-08-15 and
recorded it as a no-op. It may be a no-op precisely because the FSM is in the
wrong state and there is no accumulation gate — testing it apart from the
bracketing was pointless.

**Also measured from the same analysis:**
1. Stock's normal read is **interleaved** CH1/CH2 in one CS window (even samples
   CH1, odd CH2, 512 per channel), not two per-opcode reads like ours. Both work
   on this FPGA.
2. Stock shows 301 of 1024 samples (29% of the window); we now show 831 of 1023.

## A TRAP IN UPSTREAM'S REVERSE ENGINEERING: `WAVEFORM_DATA_PATH_CORRECTION.md` is wrong

That document (2026-04-09) claims stock reads the waveform not over SPI3 but
over the FSMC bus at `0x60020000` with DMA1, and that "`spi3_xfer(0xFF)` returns
a constant `0xFF` — there is no waveform data on that bus". **This is wrong, and
believing it costs a day.**

Three things refute it, two of them inside the document itself:
1. Its own table: `0x2C` = memory write, `0x2A` = column address set, `0x2B` =
   page address set — ILI9341 commands; the sizes `0x140 x 0xF0` = 320x240 = the
   screen resolution; the transfer size is width x height. That is a frame blit
   to the display, not a capture.
2. `ARCHITECTURE.md` calls `0x6001FFFE`/`0x60020000` the LCD command/data
   registers on EXMC, and the June analysis calls that DMA an "LCD framebuffer
   blit".
3. Our bench: SPI3 carries a live two-channel signal, measured at 5.00 MSa/s.

Upstream has not been told about this yet.

## The read frame: `r2` is a sample, `r1` is not. No change needed (2026-08-16)

**The question.** In its buffer read stock sends ONE echo byte and takes 1024
samples; we take two more bytes after the opcode as "status" `r1`/`r2` and take
1023. The suspicion: `r1`/`r2` are actually samples and we are cutting the start
off every window.

**Measurement, not reasoning.** Six consecutive bytes were added to the frame
table — `r1 r2 b0 b1 b2 b3`, where `b*` are raw window samples from the same
read. Plus two counters over all fresh frames: `r1nz` (frames with a non-zero
`r1`) and `jump` (frames where `|r2 - b0|` exceeds what a real edge can do in one
step).

Result over 193 frames: **`r1nz=0 jump=7`**.

1. **`r1` is not a sample.** It was `00` in all 193 frames, including when the
   signal sat on the top plateau (`b0=B0`). It is status or padding.
2. **`r2` is a sample.** It is continuous with the window in 96.4% of frames.
   The remaining 3.6% are windows that opened on a steep edge: a real fall
   `AF 97 5C 35 1D 0D 04` gives a step of `0x3B`, just above the chosen `0x30`
   threshold, so the exceptions are expected and their rate matches the chance of
   landing on an edge. A telling row: `00 11 46 6B 84 94` — `r2=0x11` and a
   monotonic rise after it.

**Consequence: the capture stays as it is.** We lose exactly ONE sample in 1024
and it sits inside the 160 that are discarded anyway. Changing it would recover
0.1% of the window at the risk of breaking alignment. Only the comment in
`fpga.h` was corrected: `r0/r1/r2` are not "three status bytes" as it had said
from the beginning.

**What this also explains:** why the "`r2` is the ring's write pointer"
hypothesis could not have worked. `r2` is an amplitude, not an address; it could
not have correlated with position in the first place.

## THE SEAM CLOSED: the engine glitches for the first 26 us of a capture (2026-08-16)

**Result: `hit=0/194` against `124/168`.** Not one glitch in 194 consecutive
fresh frames once the head and tail of the window are discarded. The change is in
`fpga.h`: `FPGA53_HEAD_SKIP=160`, `FPGA53_TAIL_SKIP=32`, in every build.

**What the defect is (from raw samples, not from the screen).** The head of the
window carries dropouts: a sample or several fall off the plateau, followed by an
exponential recovery over about ten samples. An example from `BADH`:

```
idx 32..44: AE AE AF AE AF AF AF AF AE AE AE B0 AF   plateau
idx 45:     0A                                       dropout
idx 46..55: 43 69 82 93 9D A4 A8 AB AE AD            recovery
```

A signal does not fall in one sample and rise over ten, so this is the
acquisition, not the input. Real edges in the same frame take about six samples
to fall.

**The boundary was measured by the firmware, not estimated.** The `GLITCH
max/hit/frames` counter: of 168 fresh frames, 124 (74%) carried a glitch and the
furthest start was sample **91**. With the dropout's width and its recovery that
contaminates roughly the first 130 of 1023 samples — the first **26 us of a
205 us window** — while the engine settles after arming. A skip of 160 gave
`hit=3/171` with all three remainders at sample **999**, at the other end of the
window, hence the tail trim of 32. Final: `hit=0/194`.

**Cost:** a visible window of 831 samples instead of 1023, 166 us instead of 205
(-19%).

**Why five hypotheses missed:** every one of them was a statement about the whole
window (PC0 gating, ping-pong, the read/write race, ring addressing, rotation by
`r2`), and the defect lives in its first 13%.

**Still open:** *why* the engine needs 26 us. Our change is a workaround, not an
explanation. (The guess "stock drops a head too" was tested the same day and
REFUTED — see the section above.)

## The seam: the `r2` hypothesis refuted, the defect redefined (2026-08-16)

**The fifth hypothesis (rotating the buffer by the status byte `r2`) did not hold
up.** It was tested not by eye but with the list of intervals between crossings
in every frame (`make release-2c53t-seam`, flag `FPGA53_SEAM_LOG=1`).

**The instrument.** The analyser counts crossings of the hysteresis band in
**both polarities** and puts a table `SEAM r2:first:gaps` into `DBG.TXT` — one
row per frame: `r2`, the index of the first crossing, then every interval as a
byte. Plus `BAND` (vmin-vmax and the thresholds) and `WIN16` (every 16th sample
of the window). Only frames whose window actually changed are recorded
(deduplicated by sum).

Two details of the instrument, each of which cost an iteration:
1. **Rising edges alone are not enough.** Against a simulated rotated ring
   (50 kHz, 5 MSa/s) a rising-only metric goes blind whenever the seam lands
   inside a plateau — which is exactly the reported symptom. With both polarities
   the seam is located to within 13 samples across the whole window, 27 hits out
   of 28.
2. **A "worst interval" summary lies.** The first version recorded the single
   most deviant interval, and in every frame it turned out to be in the first
   period. A summary only answers the question it was built for; the full
   interval list allowed the next question to be asked without another flash.

**What was measured (12 frames, 50 kHz on CH1, 20 ms/div):**
1. **The tail of the window is perfect everywhere.** Strict 48/52 (or 51/49)
   alternation, a period of exactly **100 samples** = 20 us -> 5.00 MSa/s
   confirmed for the third time. Not one grid fault across some 900 samples.
2. **The anomaly is only in the first one to three intervals.** Two patterns: a
   pair `22, 2` (a two-sample spike) or a stretched first plateau of `73` where
   the norm is 49-51.
3. **`r2` predicts nothing.** `r2=173`: one frame entirely clean, another with a
   corrupted head. `r2=0`: six frames with different heads, `first` wandering
   8..42. A write pointer would move the seam across the whole window.

**The defect is redefined: not a wandering seam but corruption of the window's
head.** "A plateau one and a half times too long" is the stretched first plateau
(73 against 51); "a sharp dip" is the two-sample spike. The rest of the window is
clean.

**Measured in passing: the same window is handed back four or five times in a
row** (24 reads gave 5 distinct frames). Consistent with the known 1.2-2 ms
refill ceiling.

**A dump trap was closed too:** a `DBG.TXT` larger than one cluster used to be
silently not written, leaving the old file on the volume — indistinguishable from
a fresh one. Now the dump is truncated and the `SEAM` table is printed first.

**Build trap:** assigning a whole struct (`ring[head] = row`) pulls in
`__aeabi_memcpy`, which a freestanding build has no one to link against — the row
is filled in place through a pointer.

## OPEN DEFECT (superseded): the seam in the frame. Four hypotheses, all refuted

Symptom (user observation, reproduced consistently): the plateau of a square wave
sometimes stretches by half through a sharp dip and back. One seam per frame,
position drifting.

**Tested and refuted, in order:**

1. **Reading on PC0 readiness** (`FPGA53_READ_PACED=0`). The artefact stayed and
   the frame rate dropped, exactly as on 08-13. Reverted.
2. **Stock's ping-pong** — two window reads back to back, the second one used. No
   change. (The `FPGA53_PINGPONG` flag stayed in the code, off by default.)
3. **The read/write race.** The arithmetic was pretty: the FPGA fills a window in
   205 us while at 24 MHz we read 1023 bytes in 341 us, so the writer overtook
   the reader inside every read. Stock at 60 MHz drains in 137 us and has no
   tearing. We moved the read to **DMA** and raised SPI to **48 MHz** (a window in
   about 171 us, the reader finally faster than the writer) — **the artefact
   remained**. The speed-up was kept; it is useful on its own.
4. **Ring addressing in the renderer.** The frame is 2048 samples, the stretched
   window occupies 2046, and the index was taken modulo 2048, so the visible span
   could join the end of the capture to its beginning. Wrapping was replaced by
   clamping — **the artefact remained**. The change was kept: joining the end to
   the beginning is wrong regardless.

**Conclusion: the seam is in the data itself**, in the 1023 bytes the FPGA hands
over. Neither the moment of the read, nor its speed, nor our addressing affects
it.

## THE TIMEBASE WAS BEING HUNTED ON THE WRONG BUS (2026-08-15, late evening)

**A fact from the user that turned everything around: stock happily displays
220 Hz.** So the hardware really does slow timebases, and our "ceiling of 500
points/s" applies only to OUR method (draining a 1023-byte window for one point),
not to the instrument.

The answer was in our own reverse engineering of stock
(`53t/reverse_engineering/FPGA_PROTOCOL_COMPLETE.md`):

1. **The timebase is set over USART2, not SPI3.** A three-command sequence from
   `FUN_0800bc00`: `0x26` prescaler -> `0x27` period -> `0x28` mode (the last one
   blocking, portMAX_DELAY). We spent two days looking for the divider in SPI3
   registers `0x0F/0x10/0x11`.
   > Superseded 2026-08-16: the divider IS on SPI3 after all, as the two-byte
   > command `01 <idx>`. The USART2 route may also exist, but the measured one is
   > SPI3.
2. **Stock has a separate streaming roll mode** (Case 1, `trigger_byte == 2`):
   five bytes per point — two per channel (16 bits!) plus one spare — into a ring
   buffer of **300 points**. Exactly the architecture already present in the
   port's `scope.c`. It does not drain a window per point, which is why it does
   not hit the refill ceiling.
3. **Fast timebases (index < 4) are configured over SPI3** with a single byte,
   the timebase index itself (`ms[0x2D]`, range 0x00-0x13) in its own CS window.
4. **The USART2 frame format is 10 bytes**, and our `dmm53.c` already sends
   frames in that format to the meter SoC (`DMM53_TX_LEN = 10`): `[0..1]` header,
   `[2]` cmd_hi, `[3]` cmd_lo, `[4]` device id, `[5..7]` parameters, `[8]`
   trailer `0xAA`, `[9]` checksum `(cmd_item + cmd_hi) & 0xFF`. Stock leaves 10 ms
   between frames.

**Extracted from the stock binary** (751 232 bytes, linked at 0x08007000, so the
file offset is the address minus 0x08007000):

`TIMEBASE_SAMPLE_TABLE` @ `0x0804D833` (offset `0x46833`), 20 entries for
indices 0x00-0x13:

```
1 1 1 1 1 1 1 1 1 1 1 1 1 1 3 5 9 21 41 82
```

By the reverse-engineered formula `threshold = table[idx] + 0x32` this is the
**read cadence**, not decimation: on slow timebases stock reads the panel less
often (up to 82 ticks against 1).

**A contradiction worth keeping in mind:** the same document claims the FPGA
initialisation (`0x01, 0x02, 0x06, 0x07, 0x08`) is sent over **USART2**, whereas
our five writes go over SPI3 at /256 and work (the 08-13 arm). Either both paths
are live or the document describes an earlier revision.

## The ceiling of the slow timebase found — for OUR method (2026-08-15, evening)

Build of the day: `2C53T-roll-final.bin` (DMA, SPI at 24 MHz, full window drain,
roll from 50 ms/div). Four measurements in a row got there, each removing one
suspect.

**1. SPI 12 -> 24 MHz (`FPGA_SPI_BR=1`) — the link holds.** The cost of a point
fell from 531 ticks x 3 us = 1.59 ms to 97 ticks x 10 us = 0.97 ms (not half,
because the byte-by-byte overhead's share grew). The data did not degrade: `ch1`
gives `AD-B1` on the plateau and `00-B1` on an edge, `CFG:OK` after a cold boot,
and `SPI3 CTRL1=034F` (BR=1) confirms the mode is really in force.

**2. DMA worked on the first try.** Channel 1 receive, channel 2 transmit on
DMA2 — the mapping came from the stock reverse engineering and turned out to be
right. **No new interrupt vector was needed**: a point is assembled over two
pacer ticks (one starts the transfer, the next collects it), so the timer ticks
twice as often and DMA2's IRQ number on this AT32 clone does not need to be
known. A watchdog covers a wrong mapping: 8 ticks without completion falls back
to byte polling forever, with a `POLLED` flag in the dump. It never fired.
Result: `cost` fell from 97 ticks to 0-1, i.e. the handler fits inside 10 us,
**and the USB starvation disappeared** — a dump was taken with roll running for
the first time, where before the volume would not mount at all.

**A trap on the way (one iteration):** the `fpga53_bus_busy` flag meant both "the
main loop holds the bus" and "a transfer is in flight". The state machine raised
it for the duration of a transfer and the check at the top of the function then
rejected the next tick — the very tick that was supposed to collect the transfer.
Symptom: `ROLL n=1` in ten seconds and no line drawn. The transfer state now
lives separately (`fpga53_pt_state`) and `fpga_capture_read` takes the bus with a
proper cancel.

**3. Half a window is NOT enough to re-arm.** `FPGA53_DRAIN_SAMPLES=512`: the
spread of points collapsed to `p=92-93` against `p=00-94` with the full 1023. So
the re-arm needs a **whole frame**, not a byte count, and the transfer cannot be
made cheaper by asking for less data.

**4. THE CEILING: the engine needs about 1.2-2 ms to refill a window.** The
decisive comparison with full drains, `over=0` and `cost=1` in both cases:

| step | point interval | spread of points |
|---|---|---|
| 50 ms/div (`pr=99`) | 2 ms | `p=00-94` — live data |
| 20 ms/div (`pr=39`) | 0.8 ms (CH1 every 1.2 ms) | `p=92-93` — frozen |

Neither the CPU nor the bus is involved: 1023 bytes at 24 MHz take 0.34 ms and
fit even in a 0.4 ms tick. We are simply re-reading a window the FPGA has not
refreshed yet. **Hence an honest ceiling of about 500 points/s**, i.e. 50 ms/div,
and it is a hardware one.

**What that means for 110 Hz: not by this route.** 500 points/s gives 4.5 points
per period — the "ragged square" visible on the device. The slow timebase covers
signals up to roughly 20-25 Hz, and that is its real domain.

**Where to go for the shape of fast signals: equivalent-time sampling.** The
window is an excellent sampler that simply catches a random phase every couple of
milliseconds. For a periodic signal the phase can be recovered by a software
trigger on the window's contents and the windows folded into one period. Then
110 Hz is assembled at the fast timebase's resolution, limited by signal
stability rather than by buffer refill.

## BENCH RULE: the battery stays in (2026-08-15, cost an evening)

**Running on USB alone with the battery removed breaks current-hungry operations
and masquerades as a firmware regression.** How it looked:

1. `CFG:NEW` instead of `CFG:OK` on every boot, configuration status
   `A=0x00035421`. Decoded bit by bit: **bit 0 = CRC error set, DONE_FINAL (bit
   13) not set** — the bitstream was arriving at the FPGA corrupted. That is a
   different class of failure from the old "the FPGA will not enter
   configuration": entry was happening.
2. Then a **self-flash hanging at "flashing"**, recoverable only through IAP
   (`53t/scripts/iap_flash.py`, needs sudo from a terminal).
3. Earlier still, UI hangs that we had written off as ISR overflow.
4. The bitstream in the source is intact throughout: its sha256 matches and the
   FNV-1a over the array is `F306AD0B`.

**With the battery back in, the same build gave `CFG:OK` on the first try.** The
diagnostic moral: `CFG:NEW` plus the CRC bit plus hangs during flash writes means
check power first and bisect the code second. Half an evening went into hunting a
regression in a change that did not contain one.

## Telemetry on screen and in DBG.TXT (decoding)

Line 1: `I<init> P<PC0 now> R<reads> C<forced> S<status: 4=FRAME> F<frames>
W<waits>`

Line 2: `<r0r1r2 of the CH1 window> <min-max CH1> b<min-max CH2> D<windows
identical> A<sweep byte><.|!> Q<5-byte status read> X<relay bank>Y<gain bank>`

Fields added to `DBG.TXT` on 2026-08-14 (commit `487f2a7`):
1. `ODR A/B/C/E` next to `IDR` — what the code wrote against what is on the pin.
   A mismatch means the pin is not an output, or the board holds it.
2. `CR A=<CRH>:<CRL> E=<CRH>:<CRL>` — the pin configuration nibbles of the two
   frontend ports: `1` = push-pull output 10 MHz, `4` = floating input, `8` =
   input with pull, `B` = alternate function (PA6 must read `B` when TMR13 is
   alive).
3. `MODE=<hex>` — low nibble the mode (0=DMM, 1=SCOPE, 2=GEN), high nibble the
   overlay (0=none, 1=mode menu, 2=settings). Without it a dump cannot be tied to
   a mode — two iterations were lost to that.
4. `POSE=<n>` — how many times `fpga53_scope_pose_reapply()` has run.
5. The meter's decoder line gained an `E0/E1` field — whether `frame[2].3` (the
   +10000 extension) fired.

Fields added later: `TB` (timebase now/want/ns/calls/skip/sent), `C1`/`C2`
(per-channel window, envelope, edges, period, glitches), `S1`/`S2` (decimated raw
windows), `FE` (frontend pins and GPIOD CRH), `RSW` (the relay ladder sweep).

## Builds in dist/variants/

1. `2C53T-v04-config.bin` — config entry with the 2C23T V0.4 sequence and a
   2C53T payload, bit-banged; works from a cold boot.
2. `2C53T-roll-mcupaced.bin` — the timebase through MCU pacing: roll from
   5 ms/div, a point being the mean of 8 window samples on a timer.
3. `2C53T-tsweep-timing.bin` — the same plus a sweep of `0x0F/0x10/0x11` with the
   `TSW` table in `DBG.TXT` (`make release-2c53t-tsweep`).
4. `2C53T-tbknob.bin` — the normal scope build where the timebase knob really
   changes the sample rate.
5. `2C53T-tb01.bin` — the sweep of the two-byte command `01 <idx>` over indices
   `0x00-0x13` (`make release-2c53t-tb01`). This is what found the sample-rate
   divider.
6. `2C53T-seam.bin` — plus the frame interval table `SEAM r2:first:gaps`, the
   `BAND`/`WIN16`/`HEAD`/`BADH` lines and the `GLITCH` counter (`make
   release-2c53t-seam`). Needs a periodic signal with several periods in the
   window and a fast timebase.
7. `2C53T-nohead.bin` — the normal scope build with the seam closed (head skip
   160, tail 32).
8. `2C53T-ch2fe.bin` — CH2's relay bank on the stock table plus the PD12/PD13
   coupling pins; the build that made CH2 work.
9. `2C53T-ch2render.bin` — plus the renderer fix that keeps the trace inside the
   capture.
10. `2C53T-rsweep.bin` / `2C53T-rsweep2.bin` — the relay ladder sweep, the second
    one looping with a pass counter (`make release-2c53t-rsweep`).
11. `2C53T-vdiv.bin` — the volts/div knob bound to the relay ladder, volts from
    the measured scale.

Earlier one-shot experiment builds (window order swap, frame preambles, `02 xx`
mode hypotheses) are kept in the same directory; all of them are negatives
described in the CH2 sections above.

## Publications (2026-08-12)

1. Report in issue #18 of DavidClawson/OpenScope-2C53T.
2. Reply in #22 (buttons plus trace), code in the `2c53t-port` branch of this
   fork.
3. maksidze's question about the payload — answered: the port does not upload a
   bitstream (warm handoff), so cross-loading is impossible until config entry is
   solved. (It was solved the same evening.)

## Warm start without reconfiguration (2026-08-15, build `2C53T-warmskip.bin`)

**Goal:** get rid of the mandatory cold boot after a self-flash without pulling
USB and the battery.

**Why a cold boot cannot be reached in software (re-checked against upstream's
documents):**

1. The MCU's only power lever is PC9 (power hold, `board_power_off`). With VBUS
   present, releasing PC9 does NOT kill the board — hence the hang on "Goodbye"
   when powering off with USB plugged in.
2. The pinhole reset and MENU+Power do not drop the FPGA rail.
3. There is no separate enable for the FPGA supply in the pin map; a RECONFIG_N
   pulse and the RELOAD command are both refuted.
4. Plus our own entry: the SRAM configuration survives 10-20 s without power, so
   an honest cold boot is USB out for 30-60 s.

**What was done instead.** The self-flash ends with a direct `bx` into the new
image (`fw_ram_install`) — the MCU is not even reset, the FPGA rail does not
blink, so the design in SRAM is alive. But `fpga_init_once` unconditionally
called `fpga53_v04_configure()`, i.e. reached into the config port of an already
configured part, which desynchronises the capture. Working hypothesis: a warm
reboot gave zeros in V/B/A and a dead engine not because the configuration was
missing but because we were breaking it ourselves.

Now, after a successful configuration (`A=0003F460`), the firmware writes a token
into RAM at the fixed address `0x20000000` (section `.warm_token` in
`linker.ld`, outside `.data`/`.bss`, so it survives both the updater's jump and
any soft reset; the address is fixed because one build writes the token and the
next one reads it). At boot the token is checked, and if it is intact the config
port is not touched at all and we go straight to the five arm writes. Three
independent invalidators: a power loss erases SRAM; the POR flag in `RCC_CSR`
(the flags are cleared every boot, so a set POR means power really went away);
and a fingerprint of the bitstream, in case the build carries a different one.

### RESULT (2026-08-15, night): CONFIRMED ON THE DEVICE, the cold boot is out of the loop

The check went faster than planned: at flashing time the device already stood
configured from a cold boot (the dump left on the volume read `V=8120681B
B=80039020 A=8003F460`, 436 frames), so instead of "flash and kill a working
configuration" a `2C53T-warmseed.bin` was built (`-DFPGA53_WARM_SEED=1`): its
first run takes as given what the telemetry already shows, does not touch the
config port, and immediately writes a real token.

**Pass 1 — the seed build, right after the jump (no cold boot):**

```
I1 P0 R914 C0 S4 F914 W0
r0=00 r1=00 r2=4A ch1=46-54 ch2=44-48 D0
V=00000000 B=00000000 A=00000000 WARM=1
calls init=915 cfg=1 poll=914
```

A second dump a minute later: `R3627 F3627`, `ch1=50-58`, and periodic content in
the window, `e=34 T16=488`. Frames keep coming and `C0` means not a single forced
read.

**Pass 2 — the ordinary `2C53T-warmskip.bin`, deciding by the token rather than
by assumption:**

```
I1 P0 R155 C0 S4 F155 W0
r0=00 r1=00 r2=4A ch1=4E-60 ch2=44-48 D0
V=00000000 B=00000000 A=00000000 WARM=1
calls init=156 cfg=1 poll=155
```

The counters reset (3627 -> 155), so the firmware applied; `WARM=1` means the
token written by the previous image was read by the next one at the fixed address
and worked; the engine is alive.

**Conclusion.** What broke a warm reboot was our own poke at the config port, not
a lost configuration. A cold boot is no longer needed in the iteration loop: copy
the binary to the volume, jump, live trace. An honest POR is only needed when the
bitstream changes (the fingerprint invalidates the token by itself) or when power
really went away.

Careful: if a power sag kills the FPGA configuration but the MCU's RAM survives,
the token lies — the symptom is `WARM=1` with a dead engine, cured by a real
power cycle. That makes the "battery stays in" rule more important, not less.

## The bitstream moved out of the image (2026-08-16, night): 229 KB -> 114 KB

**Why.** The 224 KB self-flash ceiling was not about code but about one constant:
of 228 197 bytes of sections, 115 638 (50.7%) were the bitstream table. The
largest function is 5.3 KB. Dieting the code would not have solved anything.

**What actually constrains us** (measured from the link map and the IAP
analysis):
1. The application slot is our own constant: `APP_BASE 0x08007000` (set by the
   factory bootloader) plus `APP_FLASH_SIZE_BYTES 0x38000`, with staging from
   `0x08040000`.
2. The MCU has 1 MB of flash (AT32F403A). 476 KB is used and about 544 KB above
   `0x08078000` is free.
3. RAM is the second and tighter wall: of 221 304 bytes, 210 917 were used
   (framebuffer 153 600 plus `fw_stage_halfwords` 14 336), leaving about 8.7 KB
   including the stack. So widening the slot is not free: the halfword array
   grows with `FW_MAX_SIZE`.

**Solution.** The bitstream moved into its own flash area
(`src/fpga_bitstream_store.h`): a 16-byte `GWBS` header plus length, fingerprint
and a check, then the payload. The host produces the file with `make
bitstream-bin` (fingerprint of the current bitstream **0x050EB695**) and copies
it onto the volume like any other file; the firmware writes it to flash. The
scope image became **114 352 bytes instead of 229 220**.

**Three things that had to be done right:**

1. **The second flash bank.** `0x08080000` is the start of bank 1 on the
   AT32F403A, with its own register set (`FMC_KEYR2 0x40022044`, `STS1
   0x4002204C`, `CTRL1 0x40022050`, `ADDR1 0x40022054`). Our writer knew only
   bank 0 and writes there would have gone silently nowhere. This was found in
   the factory IAP analysis BEFORE flashing, not after. The flash-ready wait also
   became bounded: a wrong guess about the registers now yields "the write did
   not happen" instead of an endless loop that would have needed a power cycle to
   escape — and would have repeated on every boot, since the file stays on the
   volume.
2. **Routing by content.** The firmware looks at the first four bytes of a file:
   `GWBS` means a bitstream (written straight to its place, nothing to install),
   an interrupt vector means an image. macOS is free to mangle the name; it
   cannot mangle the magic.
3. **Refusal instead of garbage.** An empty or damaged store (`BS=0`) means: do
   not configure at all. Pushing garbage into the FPGA is worse than not
   configuring — it closes the config port until the next honest power cycle and
   kills a design the part may already be running.

**Run on the device (without touching a cable):**

1. Flash `2C53T-extbits.bin` with an empty store -> `WARM=0 BS=0`, configuration
   never started, the engine alive on the old design (`R138 F138 C0 S4
   ch1=00-B8`).
2. Copy `F2C23T-FPGA-BITSTREAM.BIN` onto the volume -> the file disappears (not
   proof by itself).
3. Flash the same image again -> **`WARM=1 BS=1`**, `R144 F144 C0 S4`,
   `ch1=00-B1`. `BS=1` means the fingerprint the firmware computed over what is
   in flash matched the host's, so all 115 638 bytes arrived and writing to bank
   1 works. `WARM=1` means the token from the previous embedded-bitstream build
   stayed valid: the payload is the same, so the fingerprint is the same, and the
   embed-to-extern transition is transparent to the token.

**Cold boot verified (2026-08-16, night) — POSITIVE:**

```
V=8120681B B=00039020 A=8003F460 WARM=0 BS=1
I1 P0 R798 C0 S4 F798 W0   ch1=3F-55 ch2=45-48
```

Read as: `B=00039020` is the NV baseline before the load, so power really went
away and the part was cold (our honesty detector); `A=8003F460` (masking the 0x80
marker) is `CFG:OK` with DONE_FINAL, so the configuration took; `WARM=0` means
the token was correctly invalidated by the power loss, so the configurator really
ran rather than being skipped; `BS=1` means the store survived the power cycle.
The bitstream reached the FPGA by clocked reads out of bank 1 flash, and the
bit-bang rate held up.

**Builds:** `2C53T-extbits.bin` (114 352 bytes, the normal one),
`2C53T-embed-recovery.bin` (229 280 bytes, bitstream inside — for bringing up a
device with an unwritten store, and the way back),
`dist/F2C23T-FPGA-BITSTREAM.BIN` (115 654 bytes, the store itself). Targets:
`make release-2c53t-scope`, `make release-2c53t-embed`, `make bitstream-bin`.

## The firmware stager re-accounted by pages (2026-08-16): -14 KB of RAM

**What it was.** `fw_update.c` kept a bitmap of written halfwords —
`FW_MAX_SIZE/16` = **14 336 bytes of RAM** out of about 8.7 KB free, and the map
grew with the application slot. It served out-of-order arrival: it let the code
know a halfword had already been written and rebuild a page.

**Observation.** The only producer — the FAT reader in `usb_msc.c` — walks the
file sector by sector in strictly increasing offset. Out-of-order arrival is
impossible in this architecture; the machinery served a scenario that does not
exist.

**What it is now.** A single 32-bit watermark, "how many bytes are covered
contiguously from the start":

1. A chunk behind the watermark is a repeat: accepted if flash already agrees
   with it, refused otherwise (the page would have to be erased and there are no
   bytes in RAM to rebuild it from).
2. A chunk exactly at the watermark is the normal path: erase the page on first
   touch, write halfwords, advance the watermark.
3. A chunk ahead is a hole, which a legitimate producer never creates: refuse
   (`FW_UPDATE_ERR_ORDER`) rather than paper over it. The cost of a refusal is
   another copy; the cost of a silent hole is an image that passes the
   completeness check and does not boot.

The completeness check became `fw_covered >= fw_expected_size` and the
page-rebuild function disappeared entirely.

**Result:** `.bss` **196 585 instead of 210 917** (-14 332 bytes, free RAM about
23.9 KB instead of 8.7), and the image is 508 bytes smaller. The page map is now
one byte per 16 KB of slot, so **widening the application slot is no longer
limited by RAM**.

**How it was verified (and why the old way would not do).** The first flash of
the new build went through the old, proven stager and proves nothing. The second
went through the new code, and that is when it turned out there was **no way to
tell an applied firmware from a refused one**: the file disappears from the volume
either way, and the frame counters only say "the boot is old or young" (observed
between ~24 and ~140 frames per second, so the age cannot be reconstructed from
them).

The gap is closed: the dump became `v2` and gained the line `FW st=<state>
err=<code> b=<accepted>/<expected> seq=<counter>`. That line doubled as the
unambiguous marker — `v2` in a dump means an image only the rewritten stager
could have installed. Codes: st 0 idle, 1 staging, 2 ready, 3 error, 4 applying;
err 1 range, 2 vector, 3 order.
