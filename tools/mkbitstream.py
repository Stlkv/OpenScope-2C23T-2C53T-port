#!/usr/bin/env python3
"""Build the FPGA bitstream store file from the C header that used to carry it.

The payload is 113 KB of constant data that no longer belongs inside a 224 KB
app image. This emits it as a standalone file the device programs into its own
flash region when the file is dropped on the USB volume.

File layout (must match src/fpga_bitstream_store.h):

    0  magic       'GWBS'
    4  length      payload bytes
    8  fingerprint rolling sum, must match fpga.c's algorithm byte for byte
   12  check       ~(magic + length + fingerprint)
   16  payload

usage: mkbitstream.py <fpga_bitstream_2c53t.h> <out.bin>
"""

import re
import struct
import sys
from pathlib import Path

MAGIC = 0x53425747  # 'G','W','B','S' little-endian
MASK = 0xFFFFFFFF


def fingerprint(payload: bytes) -> int:
    """Same rolling sum as fpga53_bitstream_fingerprint() in fpga.c."""
    f = len(payload)
    for b in payload:
        f = (((f << 1) & MASK) | (f >> 31)) + b
        f &= MASK
    return f


def extract_payload(header: str) -> bytes:
    body = header[header.index("fpga_h2_cal_table"):]
    body = body[body.index("{") + 1: body.index("};")]
    return bytes(int(v, 0) for v in re.findall(r"0x[0-9a-fA-F]{1,2}", body))


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2

    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    header = src.read_text()

    declared = int(re.search(r"FPGA_H2_CAL_TABLE_SIZE\s+(\d+)u?", header).group(1))
    payload = extract_payload(header)
    if len(payload) != declared:
        raise SystemExit(f"payload is {len(payload)} bytes, header declares {declared}")

    fp = fingerprint(payload)
    blob = struct.pack("<IIII", MAGIC, len(payload), fp, (~(MAGIC + len(payload) + fp)) & MASK)
    blob += payload

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(blob)
    print(f"{dst}: {len(blob)} bytes (payload {len(payload)}, fingerprint 0x{fp:08X})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
