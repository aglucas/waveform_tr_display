#!/usr/bin/env python3
"""Decode odometer.bin from the SD card. Mirrors load_mileage() in main.c.

Two layouts are accepted, both sharing MILEAGE_MAGIC:
  v4      (60 bytes): magic + total_mi, trip_mi, charge_mi,
                      total_wh, trip_wh, charge_wh, last_soc
  current (61 bytes): v4 + a trailing save_is_pfail byte
                      (1 = last write was the PLP/power-fail fast-save,
                       0 = a normal periodic/event save)

current is a strict byte-prefix superset of v4: a 60-byte file is read as v4
with save_is_pfail defaulting to 0. A file >= 61 bytes is read as current and
any trailing bytes are ignored. Anything else the firmware rejects outright,
which zeroes the counters on next save -- this script says so too.

Usage: python3 decode_odometer.py odometer.bin
"""
import math
import struct
import sys

MAGIC = 0xA55A1234

V4_BYTES = 60
CURRENT_BYTES = 61

# field name -> (struct format char, present-from layout)
DOUBLE_FIELDS = [
    "total_mi", "trip_mi", "charge_mi",
    "total_wh", "trip_wh", "charge_wh",
    "last_soc",
]
V4_FMT = "<I" + "d" * len(DOUBLE_FIELDS)          # 4 + 7*8 = 60
CURRENT_FMT = V4_FMT + "B"                        # + save_is_pfail = 61

# Non-negative-and-finite check, mirroring vals_ok in load_mileage().
NONNEG_FIELDS = {"total_mi", "trip_mi", "charge_mi"}


def decode(path):
    data = open(path, "rb").read()
    size = len(data)

    if size >= CURRENT_BYTES:
        layout, layout_size, fmt = "current", CURRENT_BYTES, CURRENT_FMT
    elif size == V4_BYTES:
        layout, layout_size, fmt = "v4", V4_BYTES, V4_FMT
    else:
        print(f"{size} bytes: unrecognized length (need {V4_BYTES} or >= "
              f"{CURRENT_BYTES}) -- firmware would REJECT this and reset the "
              f"counters to 0 on the next save.")
        return

    fields = struct.unpack(fmt, data[:layout_size])
    magic = fields[0]
    doubles = fields[1:1 + len(DOUBLE_FIELDS)]
    save_is_pfail = fields[1 + len(DOUBLE_FIELDS)] if layout == "current" else None

    print(f"{size} bytes, layout={layout}({layout_size})"
          + (f"  (+{size - layout_size} trailing byte(s) ignored)"
             if size > layout_size else ""))

    magic_ok = magic == MAGIC
    print(f"magic      = 0x{magic:08X} "
          f"({'OK' if magic_ok else 'MISMATCH, expected 0xA55A1234'})")

    vals_ok = magic_ok
    for name, value in zip(DOUBLE_FIELDS, doubles):
        note = ""
        if not math.isfinite(value):
            note, vals_ok = "  <- not finite", False
        elif name in NONNEG_FIELDS and value < 0.0:
            note, vals_ok = "  <- negative", False
        elif name == "last_soc" and not (0.0 <= value <= 100.0):
            note, vals_ok = "  <- out of range 0..100", False
        print(f"{name:10s} = {value}{note}")

    if save_is_pfail is not None:
        how = "PLP / power-fail fast-save" if save_is_pfail else "normal periodic/event save"
        print(f"save_is_pfail = {save_is_pfail} ({how})")

    print(f"\nfirmware verdict: {'ACCEPT' if vals_ok else 'REJECT -- counters would reset to 0 on next save'}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    decode(sys.argv[1])
