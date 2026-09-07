#!/usr/bin/env python3
"""Host side of the port's CDC shell: live telemetry and firmware over the cable.

The device is a composite USB gadget — the MSC volume ("W25Q RAW FLASH") and
this serial port are the same cable and the same device, so nothing here
replaces the file-drop update path; it sits beside it.

    tools/cdc_bench.py mon --period 500 --log dbg.txt
    tools/cdc_bench.py cmd "fw"
    tools/cdc_bench.py flash dist/F2C23T-...-SCOPE-08007000.bin --slot b --swap

`flash` speaks the SAME wire protocol as the loader this project contributed
upstream (DavidClawson/OpenScope-2C53T#29): `fwload <size> <crc32hex> [a|b]`,
raw stream, `fwload: STAGED`, then `fwapply`. So the upstream host half,
53t/scripts/cdc_flash.py, drives this firmware unchanged — and this script
drives theirs. Staging verifies the CRC on the device BEFORE the manifest is
written, so a torn transfer cannot leave a slot that looks installable.
"""

from __future__ import annotations

import argparse
import glob
import re
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


def find_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    candidates = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not candidates:
        sys.exit("no CDC port found (looked for /dev/tty.usbmodem*, /dev/ttyACM*)")
    if len(candidates) > 1:
        print(f"# several ports, using {candidates[0]}: {candidates}", file=sys.stderr)
    return candidates[0]


def open_port(path: str) -> serial.Serial:
    # The device ignores baud (it is a virtual port), but DTR is what tells the
    # shell a session is open.
    port = serial.Serial(path, 115200, timeout=0.2, write_timeout=5)
    port.dtr = True
    time.sleep(0.15)
    port.reset_input_buffer()
    return port


def send_line(port: serial.Serial, text: str) -> None:
    port.write((text + "\r").encode())
    port.flush()


def read_until_quiet(port: serial.Serial, quiet: float = 0.35, limit: float = 5.0) -> str:
    """Read until the device stops talking for `quiet` seconds."""
    out = bytearray()
    last = time.monotonic()
    deadline = last + limit
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            out += chunk
            last = time.monotonic()
        elif time.monotonic() - last >= quiet:
            break
    return out.decode("ascii", "replace")


def cmd_cmd(args) -> int:
    with open_port(find_port(args.port)) as port:
        read_until_quiet(port, quiet=0.2, limit=1.0)   # swallow the greeting
        send_line(port, args.text)
        sys.stdout.write(read_until_quiet(port))
    return 0


def cmd_mon(args) -> int:
    log = open(args.log, "w") if args.log else None
    with open_port(find_port(args.port)) as port:
        read_until_quiet(port, quiet=0.2, limit=1.0)
        send_line(port, f"mon {args.period}")
        try:
            while True:
                chunk = port.read(512)
                if not chunk:
                    continue
                text = chunk.decode("ascii", "replace")
                sys.stdout.write(text)
                sys.stdout.flush()
                if log:
                    log.write(text)
                    log.flush()
        except KeyboardInterrupt:
            send_line(port, "mon off")
            time.sleep(0.2)
            print()
        finally:
            if log:
                log.close()
    return 0


C1_RE = re.compile(
    r"^C(?P<ch>[12]) w=(?P<wmin>[0-9A-F]{2})-(?P<wmax>[0-9A-F]{2})"
    r" env=(?P<emin>[0-9A-F]{2})-(?P<emax>[0-9A-F]{2})"
    r" e=(?P<edges>\d+) T16=(?P<t16>\d+)"
    r" fst=(?P<fmin>\d+)-(?P<fmax>\d+)/(?P<fn>\d+)"
    r" G=\d+/\d+/(?P<frames>\d+)",
    re.M,
)


def cmd_trigsweep(args) -> int:
    """Sweep scope-engine register 0x08 and tabulate the window's phase.

    The 2026-09-06 sweep could not tell "the register does nothing" from "a
    flat input has no phase to align": with no signal on CH1 every value gave
    the same 62-63 frames per 3 s and the same envelope. The number that
    separates the two is the spread of the first rising crossing (`fst` in the
    dump) read against the period (`T16`/16) measured in the same frames:

        spread ~ one period  -> the window starts wherever the refill left it
        spread ~ a few samples -> the window is aligned to the input

    So the pass/fail line is fixed before the run: any value of 0x08 whose
    spread/period ratio drops well under 1.0 while a neighbouring value sits
    near 1.0 is the register doing something. Every value near 1.0 is a
    NEGATIVE result on a live input - which the flat-input run could not be.
    """
    values = [int(v, 0) for v in args.values.split(",")] if args.values else \
        list(range(0, 256, args.step))
    with open_port(find_port(args.port)) as port:
        read_until_quiet(port, quiet=0.2, limit=1.0)
        print("reg   ch  w      fst_min-fst_max  spread  T16/16  spr/per"
              "     n   fresh/s")
        for value in values:
            send_line(port, f"trigreg {value}")
            read_until_quiet(port, quiet=0.2, limit=2.0)
            send_line(port, "dbg")                    # clears the accumulators
            before = {m.group("ch"): int(m.group("frames"))
                      for m in C1_RE.finditer(
                          read_until_quiet(port, quiet=0.4, limit=3.0))}
            started = time.monotonic()
            time.sleep(args.dwell)
            send_line(port, "dbg")                    # the frames since then
            text = read_until_quiet(port, quiet=0.4, limit=3.0)
            elapsed = time.monotonic() - started
            rows = list(C1_RE.finditer(text))
            if not rows:
                print(f"0x{value:02X}  -- no dump")
                continue
            for m in rows:
                ch = m.group("ch")
                if args.channel and ch != args.channel:
                    continue
                fmin, fmax = int(m.group("fmin")), int(m.group("fmax"))
                n, t16 = int(m.group("fn")), int(m.group("t16"))
                spread = fmax - fmin
                period = t16 / 16.0
                ratio = f"{spread / period:6.3f}" if period else "   n/a"
                # The second observable, and the one that survives a flat
                # input: `fst` needs two crossings to mean anything, the fresh-
                # frame rate needs none. It is a READ rate, not the engine's
                # refill rate - the window is 205 us at 5 MSa/s and we read it
                # about 15 times a second - so it counts reads that saw new
                # content, and it falls when the engine holds a window longer.
                fps = (int(m.group("frames")) - before.get(ch, 0)) / elapsed
                w = f"{m.group('wmin')}-{m.group('wmax')}"
                print(f"0x{value:02X}  C{ch}  {w}  {fmin:6d}-{fmax:<6d}"
                      f"  {spread:6d}  {period:6.2f}  {ratio}  {n:4d}  {fps:6.2f}")
        send_line(port, "mon off")
    return 0


def cmd_flash(args) -> int:
    payload = open(args.image, "rb").read()
    if len(payload) % 2:
        payload += b"\xff"      # the device refuses an odd size
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    slot = args.slot.lower()
    print(f"# {args.image}: {len(payload)} bytes, crc32 {crc:08X} -> slot {slot}")

    with open_port(find_port(args.port)) as port:
        read_until_quiet(port, quiet=0.2, limit=1.0)
        # Same command, same argument order, same tokens as the loader this
        # project shipped upstream in PR #29 — see scripts/cdc_flash.py there.
        send_line(port, f"fwload {len(payload)} {crc:08X} {slot}")
        reply = read_until_quiet(port, quiet=0.3, limit=3.0)
        sys.stdout.write(reply)
        if "GO " not in reply:
            return 1

        # 64-byte writes match the endpoint; the device NAKs when its ring is
        # full, so this is flow-controlled by USB and needs no pacing of its
        # own. A 4 KB sector write on the W25Q stalls it briefly — hence the
        # generous write timeout on the port.
        sent = 0
        started = time.monotonic()
        while sent < len(payload):
            n = port.write(payload[sent:sent + 64])
            sent += n or 0
            if args.progress and sent % 8192 < 64:
                pct = 100.0 * sent / len(payload)
                print(f"\r# {sent}/{len(payload)} ({pct:.0f}%)", end="", file=sys.stderr)
        port.flush()
        if args.progress:
            print(file=sys.stderr)
        elapsed = time.monotonic() - started
        print(f"# streamed in {elapsed:.1f}s ({len(payload)/max(elapsed,0.001)/1024:.0f} KB/s)")

        verdict = read_until_quiet(port, quiet=0.5, limit=30.0)
        sys.stdout.write(verdict)
        if "STAGED" not in verdict:
            return 1

        if args.swap:
            send_line(port, "fwapply")
            sys.stdout.write(read_until_quiet(port, quiet=0.5, limit=5.0))
            print("# the device resets into the new image; the port will vanish")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (default: first tty.usbmodem*)")
    sub = ap.add_subparsers(dest="action", required=True)

    p = sub.add_parser("cmd", help="run one shell command and print the reply")
    p.add_argument("text")
    p.set_defaults(func=cmd_cmd)

    p = sub.add_parser("mon", help="stream the telemetry dump until Ctrl-C")
    p.add_argument("--period", type=int, default=500, help="ms between dumps")
    p.add_argument("--log", help="also write the stream to this file")
    p.set_defaults(func=cmd_mon)

    p = sub.add_parser("trigsweep",
                       help="sweep scope reg 0x08 and tabulate window phase")
    p.add_argument("--values", help="comma list, e.g. 0x00,0x2D,0xAD,0xFF")
    p.add_argument("--step", type=int, default=16, help="sweep step when no list")
    p.add_argument("--dwell", type=float, default=1.5, help="s of frames per value")
    p.add_argument("--channel", choices=["1", "2"], default="1")
    p.set_defaults(func=cmd_trigsweep)

    p = sub.add_parser("flash", help="stream an image into a W25Q cache slot")
    p.add_argument("image")
    p.add_argument("--slot", default="b", choices=["a", "A", "b", "B"])
    p.add_argument("--swap", action="store_true", help="install it afterwards")
    p.add_argument("--progress", action="store_true", default=True)
    p.set_defaults(func=cmd_flash)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
