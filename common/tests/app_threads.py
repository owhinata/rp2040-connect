#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Checks the table of the application's threads (common/imu/app_init.c,
app_threads_start()) against the sources.

Every K_THREAD_DEFINE(name, ...) of the sources of the builds that start
their threads from the table (the files of common/, freertos/ and nuttx/
that Git tracks or would add; not the tests, among them the test of the
compatibility layer's kernel, which starts its own) must be in the table once, as APP_THREAD_START(name), and every entry must
name such a thread.  Each entry is under one #ifdef CONFIG_<X>, the option
of the module that defines the thread, and in each build the option must be
set exactly when the build compiles the defining source:

- FreeRTOS: the .cmake file (in a port/pico_sdk/ directory of common/, shared
  by the builds on the Pico SDK, or the application's CMakeLists.txt in
  freertos/) that lists the source defines CONFIG_<X>=1; no such build file
  defines it when none lists the source;
- NuttX: X is an option of the NuttX build's Kconfig and the Makefile that
  lists the source, or the Make.defs beside it, tests CONFIG_<X>; X is no
  option of the NuttX build when no NuttX build file lists the source.

Run from anywhere; prints one "ok" line per check and exits 1 on a
mismatch.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TABLE = "common/imu/app_init.c"


def tracked():
    out = subprocess.run(["git", "-C", ROOT, "ls-files", "--cached", "--others",
                          "--exclude-standard", "common", "freertos", "nuttx"],
                         capture_output=True, text=True, check=True).stdout
    # A tracked file deleted from the working tree is no longer a source.
    return [path for path in out.split() if os.path.isfile(os.path.join(ROOT, path))]


def read(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return f.read()


def table_entries():
    """(name, guard) of each APP_THREAD_START() in app_threads_start()."""
    text = read(TABLE)
    body = text[text.index("void app_threads_start(void)"):]
    entries = []
    guards = []
    for line in body.splitlines():
        m = re.match(r"\s*#\s*ifdef\s+(\w+)", line)
        if m:
            guards.append(m.group(1))
            continue
        if re.match(r"\s*#\s*if", line):
            guards.append(None)
            continue
        if re.match(r"\s*#\s*endif", line):
            guards.pop()
            continue
        m = re.match(r"\s*APP_THREAD_START\((\w+)\);", line)
        if m:
            guard = guards[-1] if len(guards) == 1 else None
            entries.append((m.group(1), guard))
    return entries


def code_lines(text):
    """The lines of a build file without its comments."""
    return [line for line in text.splitlines() if not line.lstrip().startswith("#")]


def main():
    files = tracked()
    errors = []

    defs = {}
    for path in files:
        if not path.endswith(".c") or "/tests/" in path or path.startswith("common/tests/"):
            continue
        for m in re.finditer(r"^\s*K_THREAD_DEFINE\((\w+)", read(path), re.M):
            if m.group(1) in defs:
                errors.append(f"{m.group(1)} defined in {defs[m.group(1)]} and {path}")
            defs[m.group(1)] = path

    entries = table_entries()
    names = [name for name, _ in entries]
    for name in sorted(set(n for n in names if names.count(n) > 1)):
        errors.append(f"{name} is in the table more than once")
    for name in sorted(set(defs) - set(names)):
        errors.append(f"{name} ({defs[name]}) is not in the table of {TABLE}")
    for name in sorted(set(names) - set(defs)):
        errors.append(f"{name} is in the table but no source defines it")
    for name, guard in entries:
        if guard is None or not guard.startswith("CONFIG_"):
            errors.append(f"{name}: not under one #ifdef CONFIG_<option>")
    if not errors:
        print(f"ok  table names the {len(defs)} threads of the sources")

    f_files = [p for p in files
               if (p.startswith("freertos/") or (p.startswith("common/") and "/port/pico_sdk/" in p))
               and (p.endswith(".cmake") or p.endswith("CMakeLists.txt"))]
    n_files = [p for p in files if p.startswith("nuttx/") and
               os.path.basename(p) in ("Makefile", "Make.defs")]
    n_options = set()
    for path in files:
        if path.startswith("nuttx/") and os.path.basename(path).startswith("Kconfig"):
            n_options.update(re.findall(r"^\s*(?:menu)?config\s+(\w+)", read(path), re.M))

    f_started = n_started = 0
    for name, guard in entries:
        if name not in defs or guard is None:
            continue
        base = os.path.basename(defs[name])
        option = guard[len("CONFIG_"):]

        lists = [p for p in f_files
                 if re.search(r"/" + re.escape(base) + r"\b", "\n".join(code_lines(read(p))))]
        defines = [p for p in f_files if re.search(r"\b" + guard + r"=1\b", read(p))]
        if lists:
            if not set(lists) & set(defines):
                errors.append(f"FreeRTOS: {', '.join(lists)} builds {base} "
                              f"but does not define {guard}=1 ({name})")
            f_started += 1
        elif defines:
            errors.append(f"FreeRTOS: {', '.join(defines)} defines {guard} "
                          f"but no FreeRTOS build file lists {base} ({name})")

        lists = [p for p in n_files
                 if re.search(r"\b" + re.escape(base) + r"\b", "\n".join(code_lines(read(p))))]
        if lists:
            tested = False
            for p in lists:
                for q in (p, os.path.join(os.path.dirname(p), "Make.defs")):
                    if q in files and "$(" + guard + ")" in read(q):
                        tested = True
            if option not in n_options:
                errors.append(f"NuttX: {guard} is no option of the NuttX build ({name})")
            elif not tested:
                errors.append(f"NuttX: {', '.join(lists)} builds {base} "
                              f"without testing {guard} ({name})")
            n_started += 1
        elif option in n_options:
            errors.append(f"NuttX: {guard} is an option but no NuttX build file "
                          f"lists {base} ({name})")

    for error in errors:
        print("FAIL " + error)
    if errors:
        return 1
    print(f"ok  FreeRTOS starts {f_started}, NuttX {n_started}, each under its module's option")
    return 0


if __name__ == "__main__":
    sys.exit(main())
