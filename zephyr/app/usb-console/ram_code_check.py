#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The check after the link of ram_code.cmake.

Every listed source is built (compile_commands.json), the code (.text*) of
its object that is in the image is in RAM and none of it in the flash, no
code of another object is in RAM (the long-branch stubs of the linker
excepted), and at least --min-free bytes of RAM are left after the image.
On success it prints what is in RAM; on a failure it stops the build.

The map names an object as "<archive>(<member>)", and two objects of one
library can have the same member name (sockets.c of the sockets and of the
network shell), so an object's sections are told by the archive, the member
and the section names that its object file defines.
"""

import argparse
import json
import re
import sys
from pathlib import Path

from elftools.elf.elffile import ELFFile

RAM_START = 0x20000000
RAM_END = 0x20042000


def text_sections(obj):
    with open(obj, 'rb') as f:
        return {s.name for s in ELFFile(f).iter_sections()
                if (s.name == '.text' or s.name.startswith('.text.'))
                and s['sh_size'] > 0}


def map_key(output):
    # "<dir>/CMakeFiles/<library>.dir/<path>.obj" goes into lib<library>.a.
    m = re.match(r'^(?:.*/)?CMakeFiles/([^/]+)\.dir/(?:.*/)?([^/]+)$', output)
    return (f'lib{m.group(1)}.a', m.group(2)) if m else None


def read_map(path):
    """The input sections of the map: (section, address, size, object)."""
    lines = Path(path).read_text().splitlines()
    regions = {}
    start = None
    for i, line in enumerate(lines):
        m = re.match(r'^(\S+)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)', line)
        if m and start is None:
            regions[m.group(1)] = (int(m.group(2), 16), int(m.group(3), 16))
        if line.startswith('Linker script and memory map'):
            start = i
            break
    sections = []
    ram_end = None
    name = None
    for line in lines[start:]:
        m = re.match(r'^ (\.\S+)$', line)
        if m:
            name = m.group(1)
            continue
        m = re.match(r'^ (\.\S+)?\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$', line)
        if m:
            if m.group(1):
                name = m.group(1)
            sections.append((name, int(m.group(2), 16), int(m.group(3), 16), m.group(4)))
            continue
        m = re.match(r'^\s+0x([0-9a-f]+)\s+_image_ram_end = \.', line)
        if m:
            ram_end = int(m.group(1), 16)
    return regions, sections, ram_end


def archive_member(obj):
    m = re.match(r'^(?:.*/)?([^/(]+)\(([^)]+)\)$', obj)
    return (m.group(1), m.group(2)) if m else None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--build-dir', required=True)
    parser.add_argument('--elf', required=True)
    parser.add_argument('--sources', required=True)
    parser.add_argument('--min-free', type=int, required=True)
    args = parser.parse_args()

    build = Path(args.build_dir)
    sources = [s for s in Path(args.sources).read_text().splitlines() if s]
    outputs = {}
    for entry in json.loads((build / 'compile_commands.json').read_text()):
        outputs.setdefault(str(Path(entry['file']).resolve()), []).append(entry['output'])

    errors = []
    listed = {}  # (archive, member) -> {section names}
    for source in sources:
        built = outputs.get(str(Path(source).resolve()), [])
        if len(built) != 1:
            errors.append(f'{source}: built {len(built)} times, not once')
            continue
        key = map_key(built[0])
        listed.setdefault(key, set()).update(text_sections(build / built[0]))

    regions, sections, ram_end = read_map(Path(args.elf).with_suffix('.map'))
    in_ram = {}
    in_flash = {}
    stubs_ram = stubs_flash = 0
    unlisted = set()
    for name, address, size, obj in sections:
        if size == 0 or not (name == '.text' or name.startswith('.text')):
            continue
        ram = RAM_START <= address < RAM_END
        if obj == 'linker stubs':
            if ram:
                stubs_ram += size
            else:
                stubs_flash += size
            continue
        key = archive_member(obj)
        if key in listed and name in listed[key]:
            (in_ram if ram else in_flash).setdefault(key, []).append((name, size))
        elif ram:
            unlisted.add(obj)

    for key in listed:
        if key in in_flash:
            names = ', '.join(n for n, _ in in_flash[key])
            errors.append(f'{key[0]}({key[1]}): code in the flash ({names})')
        elif key not in in_ram:
            errors.append(f'{key[0]}({key[1]}): no code in RAM')
    for obj in sorted(unlisted):
        errors.append(f'{obj}: code in RAM, not listed')

    origin, length = regions.get('RAM', (RAM_START, RAM_END - RAM_START))
    if ram_end is None:
        errors.append('no _image_ram_end in the map')
        free = 0
    else:
        free = origin + length - ram_end
        if free < args.min_free:
            errors.append(f'{free} bytes of RAM are left, fewer than {args.min_free}')

    code = sum(size for v in in_ram.values() for _, size in v)
    functions = sum(len(v) for v in in_ram.values())
    if errors:
        print('ram_code.cmake:\n  ' + '\n  '.join(errors), file=sys.stderr)
        return 1
    print(f'ram_code: {len(in_ram)} objects, {functions} sections, {code} bytes of code in RAM, '
          f'{stubs_ram} bytes of stubs in RAM, {stubs_flash} in the flash; '
          f'{free} bytes of RAM left')
    return 0


if __name__ == '__main__':
    sys.exit(main())
