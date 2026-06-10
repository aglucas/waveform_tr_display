#!/usr/bin/env python3
"""Convert an MP3 file to a C source file for boot_sound_data.c.

Usage:
    python3 tools/mp3_to_c.py path/to/sound.mp3 > main/boot_sound_data.c
"""
import sys, os

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} input.mp3", file=sys.stderr)
    sys.exit(1)

mp3_path = sys.argv[1]
with open(mp3_path, "rb") as f:
    data = f.read()

COLS = 16
print('#include "boot_sound_data.h"\n')
print(f"/* Source: {os.path.basename(mp3_path)} ({len(data)} bytes) */")
print(f"const uint8_t boot_sound_mp3[] = {{")
for i in range(0, len(data), COLS):
    chunk = data[i:i+COLS]
    print("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",")
print("};")
print(f"const size_t boot_sound_mp3_len = {len(data)};")
