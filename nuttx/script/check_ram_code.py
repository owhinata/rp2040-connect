#!/usr/bin/env python3
"""Check the code in RAM after the link (nuttx/script/build.sh runs it).

The board's linker script (scripts/arduino-nano-rp2040-connect-flash.ld)
lists, in .text's EXCLUDE_FILE between "RAM code begin" and "RAM code end",
the objects whose code (.text, .text.*) goes to .data, which the start-up
code copies to RAM.  This script reads that list, the link map and the
ELF, and fails when:

- a listed pattern matches no object with code, or a listed object's code
  is not in RAM, or code that is not listed is in RAM (only the
  .ram_code.* sections of the key-value flash routine are);
- code in RAM runs before .data is copied: what __start() calls before the
  copy (rp2040_clockconfig(), rp2040_boardearlyinitialize(),
  rp2040_lowsetup()) and what they call, leaving out the assertion paths;
- code in RAM executes "svc 0": on ARMv6-M an svc with interrupts masked
  (every context switch from a critical section) becomes a hard fault,
  which arch/arm/src/armv6-m/arm_hardfault.c forwards to the SVCall
  handler only when the PC is within _stext.._etext;
- code in RAM is on the fault path (FAULT_ROOTS and what they call);
- the heap (from the end of the IDLE stack after .bss to the end of RAM)
  is smaller than HEAP_MIN.

It prints the RAM code's size, the number of long-branch veneers (between
RAM and flash) and the heap.
"""

import argparse
import fnmatch
import re
import subprocess
import sys

# The heap holds every stack that is not a static array (the init task and
# the USB shell, the Telnet daemon and session, nina_event, each command
# while it runs, iperf's two threads, the NINA passthrough mode's two
# threads and its 16 KiB ring) and the TCBs, task groups and file lists.
# Their worst case together is estimated at about 55 KiB (see "Code in
# RAM" in nuttx/drivers/README.md); the minimum keeps about 25 KiB more
# for fragmentation and for the ring's 16 KiB in one piece.

HEAP_MIN = 80 * 1024

RAM_START = 0x20000000

# __start()'s direct calls: before the copy of .data, then after it.
START_BEFORE_COPY = ("rp2040_clockconfig", "rp2040_boardearlyinitialize",
                     "rp2040_lowsetup")
START_AFTER_COPY = ("arm_earlyserialinit", "rp2040_boardinitialize",
                    "nx_start")

# Not followed from the early code: the assertion and panic paths.
EARLY_STOP = ("_assert", "__assert", "abort")

# The fault path: the exception entry and dispatch that a hard fault goes
# through, the hard fault handler, the assertion and panic paths.  It
# stays in flash, so that a fault report does not depend on code in RAM
# that the fault may have overwritten.
FAULT_ROOTS = ("exception_common", "arm_doirq", "irq_dispatch",
               "arm_hardfault", "_assert", "__assert", "abort")


def fail(errors):
    print("check_ram_code.py: the code in RAM is not as listed:",
          file=sys.stderr)
    for e in errors:
        print("  " + e, file=sys.stderr)
    sys.exit(1)


def read_list(path):
    text = open(path).read()
    m = re.search(r"/\* RAM code begin \*/(.*?)/\* RAM code end \*/", text,
                  re.S)
    if not m:
        fail(["no \"RAM code begin\" ... \"RAM code end\" in " + path])
    body = re.sub(r"/\*.*?\*/", " ", m.group(1), flags=re.S)
    patterns = body.split()
    for p in patterns:
        if ":" not in p:
            fail([f"{p}: not an archive:object pattern ({path})"])
    return patterns


def read_map(path):
    """The input sections with code: (address, size, archive, member)."""
    sections = []
    mapped = False
    name = None
    line_re = re.compile(
        r"^ (\.\S+)?\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$")
    for line in open(path):
        if not mapped:
            mapped = line.startswith("Linker script and memory map")
            continue
        m = re.match(r"^ (\.\S+)$", line.rstrip("\n"))
        if m:
            name = m.group(1)
            continue
        m = line_re.match(line.rstrip("\n"))
        if not m:
            if not line.startswith("  "):
                name = None
            continue
        if m.group(1):
            name = m.group(1)
        size = int(m.group(3), 16)
        if name is None or size == 0:
            continue
        if not (name == ".text" or name.startswith(".text.")):
            continue
        obj = m.group(4)
        am = re.match(r"^(.*)\((.*)\)$", obj)
        archive, member = (am.group(1), am.group(2)) if am else ("", obj)
        sections.append((int(m.group(2), 16), size, archive, member))
    return sections


def read_elf(path):
    """Symbols, functions with their direct calls and svc, veneers.

    The direct calls are the bl instructions of the disassembly; calls
    through pointers are not seen.  objdump also disassembles the data
    of .data, so svc is looked for in functions only.
    """
    syms = {}
    nm = subprocess.run(["arm-none-eabi-nm", path], check=True,
                        capture_output=True, text=True).stdout
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 3:
            syms[parts[2]] = int(parts[0], 16)
    elf = subprocess.run(["arm-none-eabi-readelf", "-sW", path], check=True,
                         capture_output=True, text=True).stdout
    func_names = set()
    for line in elf.splitlines():
        parts = line.split()
        if len(parts) >= 8 and parts[3] == "FUNC":
            func_names.add(parts[7])
    dis = subprocess.run(["arm-none-eabi-objdump", "-d", path], check=True,
                         capture_output=True, text=True).stdout
    funcs = {}
    calls = {}
    svc = set()
    current = None
    for line in dis.splitlines():
        m = re.match(r"^([0-9a-f]{8}) <([^>]+)>:$", line)
        if m:
            current = m.group(2)
            funcs[current] = int(m.group(1), 16)
            calls.setdefault(current, set())
            continue
        if current is None:
            continue
        if current in func_names and re.search(r"\tsvc\t", line):
            svc.add(current)
        m = re.search(r"\tbl\s+[0-9a-f]+ <([^>+]+)>", line)
        if m:
            # A call through a long-branch veneer is a call of its target.
            callee = m.group(1)
            v = re.match(r"^__(.+)_veneer$", callee)
            calls[current].add(v.group(1) if v else callee)
    veneers = [s for s in funcs if s.endswith("_veneer")]
    return syms, funcs, calls, svc, veneers


def config_value(path, name):
    for line in open(path):
        if line.startswith(name + "="):
            return int(line.split("=", 1)[1].strip().strip('"'), 0)
    fail([f"{name} is not set in {path}"])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ld", required=True, help="the board's linker script")
    ap.add_argument("--map", required=True, help="nuttx.map")
    ap.add_argument("--elf", required=True, help="nuttx (the ELF)")
    ap.add_argument("--config", required=True, help=".config")
    args = ap.parse_args()

    ram_end = (config_value(args.config, "CONFIG_RAM_START") +
               config_value(args.config, "CONFIG_RAM_SIZE"))
    idle = config_value(args.config, "CONFIG_IDLETHREAD_STACKSIZE")

    def in_ram(addr):
        return RAM_START <= addr < ram_end

    patterns = read_list(args.ld)
    sections = read_map(args.map)
    syms, funcs, calls, svc, veneers = read_elf(args.elf)
    errors = []

    def listed(archive, member):
        for p in patterns:
            arch, mem = p.split(":", 1)
            if fnmatch.fnmatchcase(archive, arch) and \
               fnmatch.fnmatchcase(member, mem):
                return p
        return None

    matched = set()
    in_flash = set()
    not_listed = set()
    ram_bytes = 0
    stub_bytes = 0
    for addr, size, archive, member in sections:
        if member == "linker stubs":
            # The long-branch veneers that ld places next to their
            # callers, in flash or in RAM.
            if in_ram(addr):
                stub_bytes += size
            continue
        p = listed(archive, member)
        where = archive.rsplit("/", 1)[-1] + "(" + member + ")"
        if p:
            matched.add(p)
            if not in_ram(addr):
                in_flash.add(where)
        elif in_ram(addr):
            not_listed.add(where)
        if in_ram(addr):
            ram_bytes += size
    for where in sorted(in_flash):
        errors.append(f"{where}: listed, but (some of) its code is in "
                      "flash")
    for where in sorted(not_listed):
        errors.append(f"{where}: its code is in RAM but not listed")
    for p in patterns:
        if p not in matched:
            errors.append(f"{p}: matches no object with code")

    def in_ram_func(name):
        return name in funcs and in_ram(funcs[name])

    def reach(roots, stop=()):
        seen = set()
        stack = list(roots)
        while stack:
            f = stack.pop()
            if f in seen or f in stop:
                continue
            seen.add(f)
            stack.extend(calls.get(f, ()))
        return seen

    # The early code: __start() and what it calls before .data is copied.

    if "__start" not in calls:
        errors.append("__start is missing from the ELF")
    else:
        expected = set(START_BEFORE_COPY + START_AFTER_COPY)
        if not calls["__start"] <= expected:
            errors.append("__start calls " +
                          ", ".join(sorted(calls["__start"] - expected)) +
                          ", which this check does not know (before or "
                          "after the copy of .data?)")
        early = reach(["__start"] + list(START_BEFORE_COPY),
                      EARLY_STOP + START_AFTER_COPY)
        for f in sorted(early):
            if in_ram_func(f):
                errors.append(f"{f}: runs before .data is copied, but is "
                              "in RAM")

    for f in FAULT_ROOTS:
        if f not in funcs:
            errors.append(f"{f} is missing from the ELF")
    for f in sorted(reach(FAULT_ROOTS)):
        if in_ram_func(f):
            errors.append(f"{f}: on the fault path, but in RAM")

    for f in sorted(svc):
        if in_ram_func(f):
            errors.append(f"{f}: executes svc, but is in RAM (the hard "
                          "fault handler forwards svc only from flash)")

    heap_start = syms.get("_ebss", 0) + idle
    heap = ram_end - heap_start
    if "_ebss" not in syms:
        errors.append("_ebss is missing from the ELF")
    elif heap < HEAP_MIN:
        errors.append(f"the heap is {heap} bytes, less than HEAP_MIN "
                      f"({HEAP_MIN})")

    if errors:
        fail(errors)

    to_ram = sum(1 for v in veneers
                 if not in_ram(funcs[v]) and in_ram(
                     syms.get(v[2:-len("_veneer")], 0)))
    print(f"RAM code: {len(matched)} patterns, {ram_bytes} bytes and "
          f"{stub_bytes} bytes of veneers; veneers: {to_ram} from flash "
          f"to RAM, {len(veneers) - to_ram} from RAM to flash; heap "
          f"{heap} bytes (minimum {HEAP_MIN})")


if __name__ == "__main__":
    main()
