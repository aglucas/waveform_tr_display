#!/usr/bin/env python3
"""Decode odometer.bin from the SD card. Mirrors load_mileage() in main.c:
the file has shipped in four layouts (legacy/intermediate/v3/current), all
sharing MILEAGE_MAGIC and each a byte-prefix of the next except v2.

Usage: python3 decode_odometer.py odometer.bin
"""
import struct
import sys

MAGIC = 0xA55A1234

# (name, size, struct format) - format follows field order on disk.
LAYOUTS = {
    20: ("legacy",       "<Idd"),          # magic, total_mi, trip_mi
    44: ("intermediate", "<Idddd"),        # magic, total_mi, trip_mi, total_wh, trip_wh, charge_wh
    52: ("v3",           "<Iddddd"),       # + charge_mi (no last_soc)
    60: ("current",      "<Idddddd"),      # + last_soc
}


def decode(path):
    data = open(path, "rb").read()
    size = len(data)

    if size >= 60:
        layout_size = 60
    elif size in LAYOUTS:
        layout_size = size
    else:
        print(f"{size} bytes: unrecognized length, refusing to guess")
        return

    name, fmt = LAYOUTS[layout_size]
    fields = struct.unpack(fmt, data[:layout_size])
    magic = fields[0]

    print(f"{size} bytes, layout={name}({layout_size})")
    print(f"magic = 0x{magic:08X} ({'OK' if magic == MAGIC else 'MISMATCH, expected 0xA55A1234'})")

    labels_by_layout = {
        "legacy":       ["total_mi", "trip_mi"],
        "intermediate": ["total_mi", "trip_mi", "total_wh", "trip_wh", "charge_wh"],
        "v3":           ["total_mi", "trip_mi", "charge_mi", "total_wh", "trip_wh", "charge_wh"],
        "current":      ["total_mi", "trip_mi", "charge_mi", "total_wh", "trip_wh", "charge_wh", "last_soc"],
    }
    for label, value in zip(labels_by_layout[name], fields[1:]):
        print(f"{label:10s} = {value}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    decode(sys.argv[1])
