#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The I2C core (common/drivers/i2c_dw) against the Zephyr build's driver.

The Zephyr build uses Zephyr's DesignWare driver with a local patch
(zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch);
the core that the NuttX and FreeRTOS builds use is a port of it, with the
differences listed at the top of i2c_dw.c.  Its DMA reads use the OS's own
DMA driver through the boundary of common/drivers/i2c_dw/port/i2c_dw_dma_os.h in
place of Zephyr's DMA API (RENAMES turns those calls into the boundary's).
This check fails when one of them changes without the other:

- values: the definitions that both headers have (state bits, interrupt
  bits and sets, register offsets, IC_DATA_CMD command bits, FIFO depth and
  RX batch, DMA limit), the SCL counts of the four speed classes at the
  build's clock, the DMA path's lower limit, FIFO thresholds, interrupt
  masks, IC_CON bit, waits and timeout, the spike lengths;
- functions: the pairs of PAIRS, with Zephyr's names turned into the core's
  (RENAMES), compared statement by statement; the differences must be those
  of i2c_dw_sync.expected.  After an intended change of either side, look at
  the new differences and write them with --update;
- declarations: the public header that the patch adds and the one of the
  core (COPIES: <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h> and
  common/drivers/i2c_dw/include/i2c_dw_rp2040_dma.h) must be the same once the
  comments and the include guards are taken out;
- the read commands: i2c_dw_rp2040_read_commands(), a static function of
  i2c_dw_rp2040_dma_priv.h in the patch and in the core, must be the same
  function, statement by statement, on the same command bits (COMMANDS);
- the patched driver needs nothing outside the tree: each of its quoted
  #include names a file of drivers/i2c (Zephyr's or the patch's), each of
  its <zephyr/...> includes a file of the tree's include/.

The patched driver is made here from Zephyr's files at the tree's HEAD and
the patch, whatever the tree's working files hold.  The build's .config and
zephyr.dts, when given, check the Kconfig values and devicetree properties
that the core takes as constants.

usage: i2c_dw_sync.py ZEPHYR_TREE PATCH [--build DIR] [--hal DIR] [--core DIR] [--update]
"""

import argparse
import difflib
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CORE = os.path.join(HERE, '..', 'drivers', 'i2c_dw')
COMMON = os.path.join(HERE, '..')
# (file of the patched tree, the same file in common/)
COPIES = [
    ('include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h',
     'drivers/i2c_dw/include/i2c_dw_rp2040_dma.h'),
]
# The read commands' builder: a static function of i2c_dw_rp2040_dma_priv.h in the patch and in
# the core, with its command bits.
COMMANDS = 'internal/i2c_dw_rp2040_dma_priv.h'
COMMANDS_FN = 'i2c_dw_rp2040_read_commands'
COMMANDS_BITS = ('RP2040_I2C_CMD_READ', 'RP2040_I2C_CMD_STOP', 'RP2040_I2C_CMD_RESTART')
EXPECTED = os.path.join(HERE, 'i2c_dw_sync.expected')
# The core's files that hold the patched driver's code and definitions (the same names; the
# headers in its internal/).
CORE_FILES = ('i2c_dw.c', 'internal/i2c_dw.h', 'internal/i2c_dw_registers.h',
              'internal/i2c_dw_rp2040.h', 'internal/i2c_dw_rp2040_dma_priv.h')

# The Kconfig symbols of the conditionals, as the Zephyr build sets them (others: off); checked
# against the build's .config when there is one.
ZEPHYR_ON = {'CONFIG_I2C_DW_RP2040_DMA', 'CONFIG_SOC_RP2040', 'CONFIG_SHELL', 'CONFIG_PINCTRL',
             'CONFIG_RESET'}
# The core with both instances (CONFIG_I2C_DW_I2C0, CONFIG_I2C_DW_I2C1), as Zephyr's driver serves
# every enabled node.
CORE_ON = {'CONFIG_I2C_DW_RP2040_DMA', 'CONFIG_I2C_DW_I2C0', 'CONFIG_I2C_DW_I2C1'}
# The devicetree's conditionals: i2c0 has clocks and is "raspberrypi,pico-i2c", not on PCIe
# (checked against the build's zephyr.dts when there is one).
ZEPHYR_DT = {'DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks)': '1',
             'DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)': '0',
             'DT_HAS_COMPAT_STATUS_OKAY(raspberrypi_pico_i2c)': '1'}

# (Zephyr function, core function): the core's are in i2c_dw.c (with its i2c_dw_rp2040*.h) or
# i2c_dw.h.
PAIRS = [
    ('i2c_dw_error_chk', 'i2c_dw_error_chk'),
    ('i2c_dw_data_ask', 'i2c_dw_data_ask'),
    ('i2c_dw_data_read', 'i2c_dw_data_read'),
    ('i2c_dw_data_send', 'i2c_dw_data_send'),
    ('i2c_dw_transfer_complete', 'i2c_dw_transfer_complete'),
    ('i2c_dw_rp2040_dma_notify', 'i2c_dw_rp2040_dma_notify'),
    ('i2c_dw_rp2040_dma_callback', 'i2c_dw_rp2040_dma_callback'),
    ('i2c_dw_rp2040_dma_stop', 'i2c_dw_rp2040_dma_stop'),
    ('i2c_dw_rp2040_dma_read', 'i2c_dw_rp2040_dma_read'),
    ('i2c_dw_rp2040_dma_get_stats', 'i2c_dw_rp2040_dma_get_stats'),
    ('i2c_dw_rp2040_dma_enabled', 'i2c_dw_rp2040_dma_enabled'),
    ('i2c_dw_rp2040_dma_set_enabled', 'i2c_dw_rp2040_dma_set_enabled'),
    ('i2c_dw_isr', 'i2c_dw_isr_handler'),
    ('i2c_dw_setup', 'i2c_dw_setup'),
    ('i2c_dw_is_busy', 'i2c_dw_is_busy'),
    ('i2c_dw_transfer', 'i2c_dw_transfer_msgs'),
    ('i2c_dw_configure', 'i2c_dw_scl_counts'),
    ('i2c_dw_init_config', 'i2c_dw_init_config'),
    ('i2c_dw_probe_hw', 'i2c_dw_probe_hw'),
]

# Zephyr's calls and names in the core's form (applied to the Zephyr side, in order).  LOG_DBG()
# goes: the build's CONFIG_I2C_LOG_LEVEL is 3 (info).
RENAMES = [
    (r'\batomic_get\(&([\w>.-]+)\)', r'\1'),
    (r'\batomic_inc\(&(\w+)\)', r'\1++'),
    (r'\batomic_add\(&(\w+), ([^)]+)\)', r'\1 += \2'),
    (r'\batomic_set\(&([\w>.-]+), ([^)]+)\)', r'\1 = \2'),
    (r'\batomic_or\(', 'rp2040_dma_atomic_or('),
    (r'\batomic_val_t\b', 'uint32_t'),
    (r'\bk_uptime_get\(\)', 'rp2040_dma_uptime_ms()'),
    (r'\bk_sleep\(K_MSEC\((\d+)\)\)', r'i2c_dw_os_msleep(\1)'),
    (r'\bk_sem_give\(', 'i2c_dw_os_sem_give('),
    (r'\bk_sem_reset\(', 'i2c_dw_os_sem_reset('),
    (r'\bk_sem_take\(&rp2040_dma_sem, sys_timepoint_timeout\(deadline\)\)',
     'i2c_dw_os_sem_take_until(&rp2040_dma_sem, deadline)'),
    (r'\bk_timepoint_t deadline = sys_timepoint_calc\(rom->transfer_timeout\)',
     'i2c_dw_os_timepoint_t deadline = i2c_dw_os_timepoint_calc(I2C_DW_TRANSFER_TIMEOUT_MS)'),
    # Zephyr's DMA API and the Pico SDK's calls of the stop: the boundary of i2c_dw_dma_os.h
    # (the setup of a channel, dma_config() with its structures, is two calls there: the
    # differences of i2c_dw_sync.expected).
    (r'\bdevice_is_ready\(dma\)', 'i2c_dw_dma_os_is_ready()'),
    (r'\bdma_request_channel\(dma, NULL\)', 'i2c_dw_dma_os_request_channel()'),
    (r'\bdma_(stop|release_channel|start)\(dma, ', r'i2c_dw_dma_os_\1('),
    (r'\bdma_channel_is_busy\(', 'i2c_dw_dma_os_is_busy('),
    (r'\bdma_irqn_acknowledge_channel\(0, (\w+)\)', r'i2c_dw_dma_os_acknowledge(\1)'),
    (r'\bsys_write32\(', 'i2c_dw_write32('),
    # Zephyr's register address type: the core's is uintptr_t, which mm_reg_t is on Arm.
    (r'\bmm_reg_t\b', 'uintptr_t'),
    (r'\bmsg->len\b', 'len'),
    (r'\bmsg->buf\b', 'buf'),
    (r'\bmin3\((\w+), (\w+), (\w+)\)', r'MIN(MIN(\1, \2), \3)'),
    (r'\bDEVICE_MMIO_GET\((dev|port)\)', 'dw->base'),
    (r'\b(dev|port)->name\b', 'i2c_dw_name(dw)'),
    (r'\b(dev|port) == DEVICE_DT_GET\(DT_NODELABEL\(i2c0\)\)', 'i2c_dw_is_i2c0(dw)'),
    (r'\b(dev|port) != DEVICE_DT_GET\(DT_NODELABEL\(i2c0\)\)', '!i2c_dw_is_i2c0(dw)'),
    (r'\bLOG_DBG\("(?:[^"\\]|\\.)*"(?: "(?:[^"\\]|\\.)*")*(?:, [^;]*)?\); ', ''),
    (r'\bi2c_dw_rp2040_controller\((dev|port)\) && ', ''),
    (r'\b(dev|port)\b', 'dw'),
]

def git_show(tree, path):
    return subprocess.run(['git', '-C', tree, 'show', 'HEAD:' + path], check=True,
                          capture_output=True, text=True).stdout


def patched_driver(tree, patch, tmp):
    """The files of drivers/i2c at the tree's HEAD with the patch applied (by name), and the
    files that the patch adds or changes (by path)."""
    d = os.path.join(tmp, 'drivers', 'i2c')
    os.makedirs(d)
    changed = sorted(set(re.findall(r'^--- a/(\S+)$', read(patch), flags=re.M)))
    for f in sorted(set(changed) | {'drivers/i2c/i2c_dw_registers.h'}):
        os.makedirs(os.path.join(tmp, os.path.dirname(f)), exist_ok=True)
        with open(os.path.join(tmp, f), 'w') as fp:
            fp.write(git_show(tree, f))
    subprocess.run(['git', 'apply', os.path.abspath(patch)], cwd=tmp, check=True)
    files = {}
    for f in os.listdir(d):
        with open(os.path.join(d, f)) as fp:
            files[f] = fp.read()
    paths = {}
    for f in set(changed) | set(re.findall(r'^\+\+\+ b/(\S+)$', read(patch), flags=re.M)):
        with open(os.path.join(tmp, f)) as fp:
            paths[f] = fp.read()
    return files, paths


def tree_files(tree, path):
    """The names of the files of a directory of the tree at HEAD."""
    out = subprocess.run(['git', '-C', tree, 'ls-tree', '-r', '--name-only', 'HEAD', path + '/'],
                         check=True, capture_output=True, text=True).stdout
    return set(out.split())


def inline_includes(text, files):
    """The local headers of the driver that i2c_dw.c includes once, in place (the patched
    driver's or the core's: files maps their names to their text)."""
    def sub(m):
        name = m.group(1)
        return files.get(name, m.group(0)) if name.startswith("i2c_dw_rp2040") else m.group(0)
    return re.sub(r'^#include "([\w.]+)"$', sub, text, flags=re.M)


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', lambda m: '\n' * m.group(0).count('\n'), text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


def declarations(text):
    """A header's text without comments, include guard and blank lines."""
    text = strip_comments(text)
    text = re.sub(r'^\s*#\s*(ifndef|define|endif)\b[^\n]*_H_?\s*$', '', text, flags=re.M)
    return [' '.join(line.split()) for line in text.splitlines() if line.strip()]


def cond_value(expr, on):
    expr = re.sub(r'defined\s*\(\s*(\w+)\s*\)|defined\s+(\w+)',
                  lambda m: '1' if (m.group(1) or m.group(2)) in on else '0', expr)
    for k, v in ZEPHYR_DT.items():
        expr = expr.replace(k, v)
    expr = re.sub(r'\bIS_ENABLED\(\s*(\w+)\s*\)', lambda m: '1' if m.group(1) in on else '0',
                  expr)
    # Other function-like macros (DT_*() of the devicetree): not this build's.
    if re.search(r'\b[A-Za-z_]\w*\s*\(', expr):
        raise SystemExit('FAIL i2c_dw_sync: cannot evaluate #if %s' % expr)
    expr = re.sub(r'\b[A-Za-z_]\w*\b',lambda m: '1' if m.group(0) in on else '0', expr)
    expr = expr.replace('&&', ' and ').replace('||', ' or ')
    expr = re.sub(r'!(?!=)', ' not ', expr)
    return bool(eval(expr, {}))


def preprocess(text, on, seen=None):
    """Keep the lines of the conditionals that hold with the symbols of on; drop the
    conditional directives.  seen collects the CONFIG_ symbols of the conditions."""
    lines = strip_comments(text).replace('\\\n', ' ').split('\n')
    out, stack = [], []  # stack: (this branch active, a branch taken, parent active)
    active = True
    for line in lines:
        m = re.match(r'\s*#\s*(ifdef|ifndef|if|elif|else|endif)\b(.*)', line)
        if not m:
            if active:
                out.append(line)
            continue
        kw, rest = m.group(1), m.group(2).strip()
        if seen is not None:
            seen.update(re.findall(r'\bCONFIG_\w+', rest))
        if kw in ('ifdef', 'ifndef', 'if'):
            if kw == 'ifdef':
                val = rest in on
            elif kw == 'ifndef':
                val = rest not in on
            else:
                val = cond_value(rest, on)
            stack.append((active, val))
            active = active and val
        elif kw == 'elif':
            parent, taken = stack[-1]
            val = not taken and cond_value(rest, on)
            stack[-1] = (parent, taken or val)
            active = parent and val
        elif kw == 'else':
            parent, taken = stack[-1]
            active = parent and not taken
            stack[-1] = (parent, True)
        else:
            parent, _ = stack.pop()
            active = parent
    return '\n'.join(out)


def function(text, name):
    """The body of the definition of name (from its first line to the closing brace)."""
    lines = text.split('\n')
    for i, line in enumerate(lines):
        if not re.match(r'^\w[\w\s*]*\b%s\(' % re.escape(name), line):
            continue
        j = i
        while lines[j] != '{' and not lines[j].rstrip().endswith(';'):
            j += 1
        if lines[j] != '{':
            continue
        k = lines.index('}', j)
        return '\n'.join(lines[i:k + 1])
    raise SystemExit('FAIL i2c_dw_sync: no definition of %s' % name)


def statements(body, renames=(), code_only=False):
    """One statement or brace per line, white space collapsed; with code_only, the renames
    leave the string literals as they are."""
    s = ' '.join(body.split())
    for pat, rep in renames:
        if code_only:
            s = ''.join(p if i % 2 else re.sub(pat, rep, p)
                        for i, p in enumerate(re.split(r'("(?:[^"\\]|\\.)*")', s)))
        else:
            s = re.sub(pat, rep, s)
    return [p.strip() for p in re.split(r'(?<=[;{}])', s) if p.strip()]


def defines(text):
    d = {}
    for m in re.finditer(r'^\s*#\s*define\s+(\w+)(\([^)]*\))?[ \t]*(.*)$', text, flags=re.M):
        d[m.group(1)] = (m.group(2), ' '.join(m.group(3).split()))
    return d


class Check:
    def __init__(self):
        self.failed = 0

    def eq(self, what, zephyr, core):
        if zephyr == core:
            print('ok  i2c_dw_sync %s: %s' % (what, zephyr))
        else:
            print('FAIL i2c_dw_sync %s: Zephyr %s, core %s' % (what, zephyr, core),
                  file=sys.stderr)
            self.failed += 1


def one(pattern, text, what):
    m = re.findall(pattern, text)
    if not m:
        raise SystemExit('FAIL i2c_dw_sync: %s not found' % what)
    return m


def read(path):
    with open(path) as fp:
        return fp.read()


def scl_counts(zh, mhz, tmp, core):
    """The SCL counts of the four speed classes, of each side, by the host's compiler."""
    names = ['I2C_STD_HCNT', 'I2C_STD_LCNT', 'I2C_FS_HCNT', 'I2C_FS_LCNT', 'I2C_FSP_HCNT',
             'I2C_FSP_LCNT', 'I2C_HS_HCNT', 'I2C_HS_LCNT', 'I2C_MIN_SCL_LCNT',
             'I2C_MIN_SCL_HCNT', 'I2C_SCL_HCNT_OFFSET', 'I2C_ENSURE_MIN_SCL_HCNT',
             'I2C_ENSURE_MIN_SCL_LCNT']
    zdef = ''.join('#define %s%s %s\n' % (n, zh[n][0] or '', zh[n][1]) for n in names if n in zh)
    zsrc = '''#include <stdio.h>
#define CONFIG_I2C_DW_CLOCK_SPEED %d
%s
#define SPK %s
#define MAX(a, b) ((a) > (b) ? (a) : (b))
int main(void)
{
	unsigned int c[4][2] = {{I2C_STD_HCNT, I2C_STD_LCNT}, {I2C_FS_HCNT, I2C_FS_LCNT},
				{I2C_FSP_HCNT, I2C_FSP_LCNT}, {I2C_HS_HCNT, I2C_HS_LCNT}};
	for (int i = 0; i < 4; i++) {
		printf("%%u/%%u ", I2C_ENSURE_MIN_SCL_HCNT(c[i][0], SPK),
		       I2C_ENSURE_MIN_SCL_LCNT(c[i][1], SPK));
	}
	return 0;
}
''' % (mhz, zdef, zh['DW_IC_SPKLEN_MIN'][1])
    csrc = '''#include <stdio.h>
#include "i2c_dw.h"
int main(void)
{
	for (unsigned int s = I2C_SPEED_STANDARD; s <= I2C_SPEED_HIGH; s++) {
		uint16_t h, l;
		i2c_dw_scl_counts(s, %d, true, &h, &l);
		printf("%%u/%%u ", h, l);
	}
	return 0;
}
''' % mhz
    res = []
    for name, src, inc in (('z', zsrc, []), ('c', csrc, ['-I', os.path.join(core, 'internal')])):
        c = os.path.join(tmp, name + '.c')
        exe = os.path.join(tmp, name)
        with open(c, 'w') as fp:
            fp.write(src)
        subprocess.run(['cc', '-std=c17', '-Wall', '-Werror', *inc, c, '-o', exe], check=True)
        res.append(subprocess.run([exe], check=True, capture_output=True,
                                  text=True).stdout.strip())
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tree')
    ap.add_argument('patch')
    ap.add_argument('--build', help="the Zephyr build's directory (zephyr/.config, zephyr.dts)")
    ap.add_argument('--hal', help='the hal_rpi_pico module (hardware/regs/i2c.h; default: '
                    'modules/hal/rpi_pico next to the tree)')
    ap.add_argument('--core', default=CORE, help='the core (default: common/drivers/i2c_dw)')
    ap.add_argument('--update', action='store_true')
    args = ap.parse_args()
    core = args.core
    ck = Check()

    with tempfile.TemporaryDirectory() as tmp:
        files, paths = patched_driver(args.tree, args.patch, tmp)
        seen = set()
        zc = preprocess(inline_includes(files['i2c_dw.c'], files), ZEPHYR_ON, seen)
        zh = defines(preprocess(files['i2c_dw.h'] + files['i2c_dw_registers.h'] +
                                files['i2c_dw_rp2040.h'] + files['i2c_dw_rp2040_dma_priv.h'],
                                ZEPHYR_ON, seen))

        # The copies, and the patched driver's includes.
        ck.eq('declarations of the patch that differ in common/ (of %d)' % len(COPIES),
              [z for z, c in COPIES
               if declarations(paths.get(z, '')) != declarations(read(os.path.join(COMMON, c)))],
              [])
        local = tree_files(args.tree, 'drivers/i2c') | {p for p in paths}
        incs = tree_files(args.tree, 'include/zephyr') | {p for p in paths}
        missing = []
        for p in sorted(p for p in paths if p.startswith('drivers/i2c/')):
            for q in re.findall(r'^\s*#\s*include\s+"([^"]+)"', paths[p], flags=re.M):
                if 'drivers/i2c/' + q not in local:
                    missing.append('%s: "%s"' % (p, q))
            for q in re.findall(r'^\s*#\s*include\s+<(zephyr/[^>]+)>', paths[p], flags=re.M):
                if 'include/' + q not in incs and not q.startswith('zephyr/syscalls/'):
                    missing.append('%s: <%s>' % (p, q))
        ck.eq("the patched driver's includes found nowhere in the tree", missing, [])
        # The read commands' builder.
        cmd = preprocess(read(os.path.join(core, COMMANDS)), CORE_ON)
        zf = statements(function(zc, COMMANDS_FN))
        ck.eq('%s() of the patch and of %s (statements that differ)' % (COMMANDS_FN, COMMANDS),
              [x for x in difflib.unified_diff(zf, statements(function(cmd, COMMANDS_FN)), n=0,
                                               lineterm='')], [])
        ck.eq('command bits of %s()' % COMMANDS_FN,
              [zh.get(n, 'none in the patch') for n in COMMANDS_BITS],
              [defines(cmd).get(n, 'none in the core') for n in COMMANDS_BITS])
        # The core's files are named as the patched driver's; i2c_dw.c includes its
        # i2c_dw_rp2040*.h once, as Zephyr's does.
        cfiles = {os.path.basename(f): read(os.path.join(core, f)) for f in CORE_FILES}
        core_c = preprocess(inline_includes(cfiles['i2c_dw.c'], cfiles) + cfiles['i2c_dw.h'],
                            CORE_ON)
        ch = defines(preprocess(cfiles['i2c_dw.h'] + cfiles['i2c_dw_registers.h'] +
                                cfiles['i2c_dw_rp2040.h'] + cfiles['i2c_dw_rp2040_dma_priv.h'],
                                CORE_ON))

        config = {}
        if args.build:
            for line in read(os.path.join(args.build, 'zephyr', '.config')).split('\n'):
                m = re.match(r'(CONFIG_\w+)=(.*)', line)
                if m:
                    config[m.group(1)] = m.group(2).strip('"')
            wrong = sorted(s for s in seen if (config.get(s) == 'y') != (s in ZEPHYR_ON))
            ck.eq("conditionals' symbols as in the build's .config (wrong ones)", wrong, [])
            ck.eq('CONFIG_I2C_LOG_LEVEL (LOG_DBG() left out)', config.get('CONFIG_I2C_LOG_LEVEL'), '3')

        # The definitions of both headers.
        common = sorted(set(zh) & set(ch))
        diff = [n for n in common if zh[n] != ch[n] and zh[n][1] != '' and
                not n.startswith(('DEFINE_', 'I2C_STD_', 'I2C_FS_', 'I2C_FSP_', 'I2C_HS_', 'I2C_MIN_SCL_',
                                  'I2C_ENSURE_MIN_SCL_'))]
        ck.eq('definitions of both headers that differ (of %d)' % len(common), diff, [])
        # The register accessors (DEFINE_*_OP() and DEFINE_MM_REG_*(): the generators differ).
        acc = [sorted(set(re.findall(r'^DEFINE_\w+\([^)]*\)', preprocess(x, on), flags=re.M)))
               for x, on in ((files['i2c_dw_registers.h'], ZEPHYR_ON),
                             (cfiles['i2c_dw_registers.h'], CORE_ON))]
        ck.eq('register accessors that differ (of %d)' % len(acc[0]),
              sorted(set(acc[0]) ^ set(acc[1])), [])
        for n in ('RP2040_I2C_FIFO_DEPTH', 'RP2040_I2C_RX_BATCH', 'I2C_DW_RP2040_DMA_MAX_READ',
                  'IC_DATA_CMD_CMD', 'IC_DATA_CMD_STOP', 'IC_DATA_CMD_RESTART',
                  'DW_IC_REG_DATA_CMD', 'DW_IC_SPKLEN_MIN'):
            ck.eq(n, zh.get(n, (None, None))[1], ch.get(n, (None, None))[1])

        # The DMA path's events.
        zr = {n: v for n, v in defines(zc).items() if n.startswith('RP2040_DMA_')}
        cr = {n: v for n, v in defines(core_c).items() if n.startswith('RP2040_DMA_')}
        ck.eq('DMA events', sorted(zr.items()), sorted(cr.items()))

        # The SCL counts (hcnt/lcnt: standard, fast, fast plus, high speed).
        mhz = int(config.get('CONFIG_I2C_DW_CLOCK_SPEED', '0') or 0)
        if not mhz:
            board = os.path.join(args.tree, 'boards', 'arduino', 'nano_connect',
                                 'Kconfig.defconfig')
            mhz = int(one(r'config I2C_DW_CLOCK_SPEED\s+default (\d+)', read(board),
                          'I2C_DW_CLOCK_SPEED')[0])
        ck.eq('CONFIG_I2C_DW_CLOCK_SPEED (MHz)', mhz, 125)
        z, c = scl_counts(zh, mhz, tmp, core)
        ck.eq('SCL counts at %d MHz' % mhz, z, c)
        # Zephyr: MAX(the devicetree's property or 0, DW_IC_SPKLEN_MIN); no property (below).
        ck.eq('spike lengths (fs, hs)',
              [one(r'\.%s_spk_len = MAX\(\(uint8_t\)DT_INST_PROP_OR\(n, %s_spike_len, 0\), '
                   r'(\w+)\)' % (s, s), zc, s + '_spk_len')[0] for s in ('fs', 'hs')],
              [ch['I2C_DW_FS_SPK_LEN'][1], ch['I2C_DW_HS_SPK_LEN'][1]])
        if args.build:
            dts = read(os.path.join(args.build, 'zephyr', 'zephyr.dts'))
            node = strip_comments(one(r'(?s)\bi2c0:[^{]*\bi2c@40044000 \{(.*?)\n\t\t\};', dts,
                                      'i2c0 in zephyr.dts')[0])
            ck.eq("i2c0's clocks and compatible (the devicetree's conditionals)",
                  [bool(re.search(r'\bclocks = ', node)),
                   bool(re.search(r'compatible = "raspberrypi,pico-i2c"', node))], [True, True])
            props = sorted(p for p in ('fs-spike-len', 'hs-spike-len', 'lcnt-offset',
                                       'hcnt-offset', 'sda-hold-tx', 'sda-hold-rx',
                                       'i2c-sda-hold-time-ns', 'zephyr,transfer-timeout-ms')
                           if re.search(r'\b%s\b' % re.escape(p), node))
            ck.eq("i2c0's devicetree properties that the core takes as absent", props, [])
            ck.eq('transfer timeout (ms)', int(config['CONFIG_I2C_TRANSFER_TIMEOUT_MS']),
                  int(ch['I2C_DW_TRANSFER_TIMEOUT_MS'][1]))
            ck.eq('clock-frequency of i2c0', one(r'clock-frequency = <\s*(\w+)\s*>', node, 'clock')[0],
                  '0x186a0')
        else:
            print('skip i2c_dw_sync .config and zephyr.dts (no build)')

        # The DMA path's values.
        zd = ' '.join(function(zc, 'i2c_dw_rp2040_dma_read').split())
        cd = ' '.join(function(core_c, 'i2c_dw_rp2040_dma_read').split())
        ck.eq('DMA lower limit', one(r'msg->len < (\d+)', zd, 'limit'),
              one(r'\blen < (\d+)', cd, 'limit'))
        ck.eq('DMA upper limit', one(r'msg->len > (\w+)', zd, 'limit'),
              one(r'\blen > (\w+)', cd, 'limit'))
        ck.eq('FIFO thresholds during DMA (value, register)',
              one(r'sys_write32\((\d+), base \+ (0x\w+)\)', zd, 'thresholds'),
              one(r'i2c_dw_write32\((\d+), base \+ (0x\w+)\)', cd, 'thresholds'))

        def mask(pattern, text, what):
            return sorted(re.findall(r'DW_INTR_STAT_\w+', one(pattern, text, what)[0]))
        ck.eq('interrupt mask during DMA',
              mask(r'write_intr_mask\((DW_INTR_STAT[^;]*), base\)', zd, 'mask'),
              mask(r'write_intr_mask\((DW_INTR_STAT[^;]*), base\)', cd, 'mask'))
        zi = ' '.join(function(zc, 'i2c_dw_isr').split())
        ci = ' '.join(function(core_c, 'i2c_dw_isr_handler').split())
        ck.eq('errors of the DMA interrupt',
              mask(r'rp2040_dma_active\)\) \{ atomic_val_t events = 0; if \(intr_stat\.raw & '
                   r'\(([^)]*)\)', zi, 'isr'),
              mask(r'rp2040_dma_active\) \{ uint32_t events = 0; if \(intr_stat\.raw & '
                   r'\(([^)]*)\)', ci, 'isr'))
        ck.eq('DMA waits (ms)', one(r'k_uptime_get\(\) \+ (\d+)', zc, 'waits'),
              one(r'rp2040_dma_uptime_ms\(\) \+ (\d+)', core_c, 'waits'))
        ck.eq('DMA timeout', one(r'sys_timepoint_calc\((rom->transfer_timeout)\)', zd, 'timeout'),
              [x.replace('I2C_DW_TRANSFER_TIMEOUT_MS', 'rom->transfer_timeout') for x in
               one(r'i2c_dw_os_timepoint_calc\((\w+)\)', cd, 'timeout')])
        zs = ' '.join(function(zc, 'i2c_dw_setup').split())
        cs = ' '.join(function(core_c, 'i2c_dw_setup').split())
        zbit = one(r'ic_con\.raw \|= (\w+);', zs, 'IC_CON')[0]
        hal = args.hal or os.path.join(args.tree, '..', 'modules', 'hal', 'rpi_pico')
        regs = read(os.path.join(hal, 'src', 'rp2040', 'hardware_regs', 'include', 'hardware',
                                 'regs', 'i2c.h'))
        val = int(one(r'#define %s\s+_u\((0x\w+)\)' % zbit, regs, zbit)[0], 16)
        zbit = 'BIT(%d)' % (val.bit_length() - 1) if val & (val - 1) == 0 else hex(val)
        ck.eq('IC_CON bit with the DMA reads', zbit, one(r'ic_con\.raw \|= ([\w()]+);', cs,
                                                         'IC_CON')[0])

        # The functions.
        lines = []
        for zname, cname in PAIRS:
            z = statements(function(zc, zname), RENAMES)
            c = statements(function(core_c, cname))
            for d in difflib.unified_diff(z, c, 'zephyr ' + zname, 'core ' + cname, n=0,
                                          lineterm=''):
                lines.append('@@' if d.startswith('@@') else d)
        actual = '\n'.join(lines) + '\n'
        if args.update:
            with open(EXPECTED, 'w') as fp:
                fp.write(actual)
            print('wrote %s' % EXPECTED)
        else:
            expected = read(EXPECTED)
            if actual == expected:
                print('ok  i2c_dw_sync functions (%d pairs, %d lines of expected differences)' %
                      (len(PAIRS), sum(1 for x in lines if x[:1] in '+-' and
                                       not x.startswith(('+++', '---')))))
            else:
                print('FAIL i2c_dw_sync functions: the differences are not those of %s:' %
                      os.path.relpath(EXPECTED), file=sys.stderr)
                sys.stderr.writelines(difflib.unified_diff(
                    expected.splitlines(True), actual.splitlines(True), 'expected', 'actual'))
                ck.failed += 1
    return 1 if ck.failed else 0


if __name__ == '__main__':
    sys.exit(main())
