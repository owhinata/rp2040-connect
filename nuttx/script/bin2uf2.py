#!/usr/bin/env python3
"""Convert a flat RP2040 flash image (nuttx.bin) to UF2.

The input is the raw image as it goes into flash at --base (0x10000000 by
default), starting with the 256-byte boot2.  The output follows the UF2
format (https://github.com/microsoft/uf2) the way the pico-sdk tools write
it for RP2040: 512-byte blocks, each carrying 256 bytes of payload for a
256-byte-aligned flash address, the "familyID present" flag and the RP2040
family ID (0xe48bff56).  The last block is padded with zeros.

Before converting, the boot2 checksum is checked: the boot ROM only runs
boot2 when the last 4 bytes of the first 256 hold the CRC-32 of the first
252 (polynomial 0x04c11db7, initial value 0xffffffff, no reflection, no
final XOR; the value pico-sdk's pad_checksum writes with -s 0xffffffff).
"""

import argparse
import os
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
UF2_BLOCK_SIZE = 512
UF2_DATA_SIZE = 476            # data[] field of a block
PAYLOAD_SIZE = 256             # what the RP2040 boot ROM expects
RP2040_FAMILY_ID = 0xE48BFF56
BOOT2_SIZE = 256


def crc32_boot2(data):
    """CRC-32 as checked by the RP2040 boot ROM (CRC-32/MPEG-2)."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def check_boot2(image):
    if len(image) < BOOT2_SIZE:
        raise ValueError(f"image is {len(image)} bytes, shorter than boot2 ({BOOT2_SIZE})")
    expected = crc32_boot2(image[:BOOT2_SIZE - 4])
    stored = struct.unpack_from("<I", image, BOOT2_SIZE - 4)[0]
    if stored != expected:
        raise ValueError(
            f"boot2 checksum mismatch: stored 0x{stored:08x}, computed 0x{expected:08x} "
            "(no boot2 at the start of the image?)")
    return stored


def make_uf2(image, base, family_id):
    if base % PAYLOAD_SIZE:
        raise ValueError(f"base 0x{base:08x} is not {PAYLOAD_SIZE}-byte aligned")
    num_blocks = (len(image) + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE
    out = bytearray()
    for block_no in range(num_blocks):
        chunk = image[block_no * PAYLOAD_SIZE:(block_no + 1) * PAYLOAD_SIZE]
        chunk = chunk + bytes(PAYLOAD_SIZE - len(chunk))
        header = struct.pack(
            "<8I",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            UF2_FLAG_FAMILY_ID_PRESENT,
            base + block_no * PAYLOAD_SIZE,
            PAYLOAD_SIZE,
            block_no,
            num_blocks,
            family_id,
        )
        data = chunk + bytes(UF2_DATA_SIZE - PAYLOAD_SIZE)
        block = header + data + struct.pack("<I", UF2_MAGIC_END)
        assert len(block) == UF2_BLOCK_SIZE
        out += block
    return bytes(out), num_blocks


def any_int(text):
    return int(text, 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", help="raw flash image (nuttx.bin)")
    parser.add_argument("output", help="UF2 file to write")
    parser.add_argument("--base", type=any_int, default=0x10000000,
                        help="flash address of the first byte (default 0x10000000)")
    parser.add_argument("--family", type=any_int, default=RP2040_FAMILY_ID,
                        help="UF2 family ID (default 0xe48bff56, RP2040)")
    parser.add_argument("--end", type=any_int, default=None,
                        help="refuse an image that reaches this address (exclusive)")
    args = parser.parse_args(argv)

    with open(args.input, "rb") as f:
        image = f.read()

    try:
        crc = check_boot2(image)
        if args.end is not None and args.base + len(image) > args.end:
            raise ValueError(
                f"image ends at 0x{args.base + len(image):08x}, beyond 0x{args.end:08x}")
        uf2, num_blocks = make_uf2(image, args.base, args.family)
    except ValueError as e:
        print(f"bin2uf2: {args.input}: {e}", file=sys.stderr)
        return 1

    tmp = args.output + ".tmp"
    with open(tmp, "wb") as f:
        f.write(uf2)
    os.replace(tmp, args.output)

    last = args.base + (num_blocks - 1) * PAYLOAD_SIZE
    print(f"bin2uf2: {len(image)} bytes -> {num_blocks} blocks, "
          f"0x{args.base:08x}-0x{last + PAYLOAD_SIZE - 1:08x}, "
          f"family 0x{args.family:08x}, boot2 CRC 0x{crc:08x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
