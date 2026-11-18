#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The I2C0 core (common/drivers/i2c_dw) against the Zephyr build's driver.

The Zephyr build uses Zephyr's DesignWare driver with a local patch
(zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch);
the core that the NuttX and FreeRTOS builds use is a port of the patched
driver, with the differences listed at the top of nano_i2c.c.  This check
fails when one of them changes without the other:

- values: the definitions that both headers have (state bits, interrupt
  bits and sets, register offsets, IC_DATA_CMD command bits, FIFO depth and
  RX batch, DMA limit), the SCL counts of the four speed classes at the
  build's clock, the DMA path's lower limit, FIFO thresholds, interrupt
  masks, IC_CON bit, waits and timeout, the spike lengths;
- functions: the pairs of PAIRS, with Zephyr's names turned into the core's
  (RENAMES), compared statement by statement; the differences must be those
  of i2c_dw_sync.expected.  After an intended change of either side, look
  at the new differences and write them with --update.

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
EXPECTED = os.path.join(HERE, 'i2c_dw_sync.expected')

# The Kconfig symbols of the conditionals, as the Zephyr build sets them (others: off); checked
# against the build's .config when there is one.
ZEPHYR_ON = {'CONFIG_I2C_DW_RP2040_DMA', 'CONFIG_SOC_RP2040', 'CONFIG_SHELL', 'CONFIG_PINCTRL',
             'CONFIG_RESET'}
CORE_ON = {'CONFIG_NANO_I2C0_DMA'}
# The devicetree's conditionals: i2c0 has clocks and is "raspberrypi,pico-i2c", not on PCIe
# (checked against the build's zephyr.dts when there is one).
ZEPHYR_DT = {'DT_ANY_INST_HAS_PROP_STATUS_OKAY(clocks)': '1',
             'DT_ANY_INST_ON_BUS_STATUS_OKAY(pcie)': '0',
             'DT_HAS_COMPAT_STATUS_OKAY(raspberrypi_pico_i2c)': '1'}

# (Zephyr function, core function): the core's are in nano_i2c.c or nano_i2c_dw.h.
PAIRS = [
    ('i2c_dw_error_chk', 'i2c_dw_error_chk'),
    ('i2c_dw_data_ask', 'i2c_dw_data_ask'),
    ('i2c_dw_data_read', 'i2c_dw_data_read'),
    ('i2c_dw_data_send', 'i2c_dw_data_send'),
    ('i2c_dw_transfer_complete', 'i2c_dw_transfer_complete'),
    ('i2c_dw_rp_dma_notify', 'i2c_dw_rp_dma_notify'),
    ('i2c_dw_rp_dma_callback', 'i2c_dw_rp_dma_callback'),
    ('i2c_dw_rp_dma_stop', 'i2c_dw_rp_dma_stop'),
    ('i2c_dw_rp_dma_read', 'i2c_dw_rp_dma_read'),
    ('nano_i2c_dma_get_stats', 'nano_i2c_dma_get_stats'),
    ('nano_i2c_dma_enabled', 'nano_i2c_dma_enabled'),
    ('nano_i2c_dma_set_enabled', 'nano_i2c_dma_set_enabled'),
    ('i2c_dw_isr', 'i2c_dw_isr_handler'),
    ('i2c_dw_setup', 'i2c_dw_setup'),
    ('i2c_dw_is_busy', 'i2c_dw_is_busy'),
    ('i2c_dw_transfer', 'i2c_dw_transfer_msgs'),
    ('i2c_dw_configure', 'nano_i2c_dw_scl_counts'),
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
    (r'\batomic_or\(', 'rp_dma_atomic_or('),
    (r'\batomic_val_t\b', 'uint32_t'),
    (r'\bk_uptime_get\(\)', 'rp_dma_uptime_ms()'),
    (r'\bk_sleep\(K_MSEC\((\d+)\)\)', r'i2c_dw_os_msleep(\1)'),
    (r'\bk_sem_give\(', 'i2c_dw_os_sem_give('),
    (r'\bk_sem_reset\(', 'i2c_dw_os_sem_reset('),
    (r'\bk_sem_take\(&dw->rp_dma_sem, sys_timepoint_timeout\(deadline\)\)',
     'i2c_dw_os_sem_take_until(&dw->rp_dma_sem, deadline)'),
    (r'\bk_timepoint_t deadline = sys_timepoint_calc\(rom->transfer_timeout\)',
     'i2c_dw_os_timepoint_t deadline = i2c_dw_os_timepoint_calc(NANO_I2C_DW_TRANSFER_TIMEOUT_MS)'),
    (r'\bdma_request_channel\(dma, NULL\)', 'nano_dma_request_channel()'),
    (r'\bdma_(stop|release_channel|config|start)\(dma, ', r'nano_dma_\1('),
    (r'\bdma_channel_is_busy\(', 'nano_dma_channel_is_busy('),
    (r'\bdma_irqn_acknowledge_channel\(', 'nano_dma_irqn_acknowledge_channel('),
    (r'\bdma_hw->abort\b', 'nano_dma_abort_pending()'),
    (r'\bstruct dma_(block_config|config)\b', r'struct nano_dma_\1'),
    (r'\bDMA_ADDR_ADJ_', 'NANO_DMA_ADDR_ADJ_'),
    (r'\bRPI_PICO_DMA_SLOT_', 'NANO_DMA_SLOT_'),
    (r'\b(MEMORY_TO_PERIPHERAL|PERIPHERAL_TO_MEMORY)\b', r'NANO_DMA_\1'),
    (r'\bsys_write32\(', 'nano_dw_write32('),
    (r'\bmsg->len\b', 'len'),
    (r'\bmsg->buf\b', 'buf'),
    (r'\bmin3\((\w+), (\w+), (\w+)\)', r'MIN(MIN(\1, \2), \3)'),
    (r'\bDEVICE_MMIO_GET\((dev|port)\)', 'dw->base'),
    (r'\b(dev|port)->name\b', 'I2C_DW_NAME'),
    (r'\bLOG_DBG\("(?:[^"\\]|\\.)*"(?: "(?:[^"\\]|\\.)*")*(?:, [^;]*)?\); ', ''),
    (r'\bi2c_dw_rp2040_controller\((dev|port)\) && ', ''),
    (r'\b(dev|port)\b', 'dw'),
]


def git_show(tree, path):
    return subprocess.run(['git', '-C', tree, 'show', 'HEAD:' + path], check=True,
                          capture_output=True, text=True).stdout


def patched_driver(tree, patch, tmp):
    """The files of drivers/i2c at the tree's HEAD with the patch applied."""
    d = os.path.join(tmp, 'drivers', 'i2c')
    os.makedirs(d)
    for f in ('i2c_dw.c', 'i2c_dw.h', 'i2c_dw_registers.h', 'Kconfig.dw'):
        with open(os.path.join(d, f), 'w') as fp:
            fp.write(git_show(tree, 'drivers/i2c/' + f))
    subprocess.run(['git', 'apply', os.path.abspath(patch)], cwd=tmp, check=True)
    files = {}
    for f in os.listdir(d):
        with open(os.path.join(d, f)) as fp:
            files[f] = fp.read()
    return files


def inline_includes(text, files):
    """The local headers of the driver that i2c_dw.c includes once, in place."""
    def sub(m):
        name = m.group(1)
        return files.get(name, m.group(0)) if name.startswith("i2c_dw_rp2040") else m.group(0)
    return re.sub(r'^#include "([\w.]+)"$', sub, text, flags=re.M)


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', lambda m: '\n' * m.group(0).count('\n'), text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


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


def statements(body, renames=()):
    """One statement or brace per line, white space collapsed."""
    s = ' '.join(body.split())
    for pat, rep in renames:
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
#include "nano_i2c_dw.h"
int main(void)
{
	for (unsigned int s = NANO_I2C_SPEED_STANDARD; s <= NANO_I2C_SPEED_HIGH; s++) {
		uint16_t h, l;
		nano_i2c_dw_scl_counts(s, %d, true, &h, &l);
		printf("%%u/%%u ", h, l);
	}
	return 0;
}
''' % mhz
    res = []
    for name, src, inc in (('z', zsrc, []), ('c', csrc, ['-I', core])):
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
        files = patched_driver(args.tree, args.patch, tmp)
        seen = set()
        zc = preprocess(inline_includes(files['i2c_dw.c'], files), ZEPHYR_ON, seen)
        zh = defines(preprocess(files['i2c_dw.h'] + files['i2c_dw_registers.h'] +
                                files['i2c_dw_rp2040.h'], ZEPHYR_ON, seen))
        core_c = preprocess(read(os.path.join(core, 'nano_i2c.c')) +
                            read(os.path.join(core, 'nano_i2c_dw.h')), CORE_ON)
        ch = defines(preprocess(read(os.path.join(core, 'nano_i2c_dw.h')) +
                                read(os.path.join(core, 'nano_i2c_dw_registers.h')), CORE_ON))

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
                             (read(os.path.join(core, 'nano_i2c_dw_registers.h')), CORE_ON))]
        ck.eq('register accessors that differ (of %d)' % len(acc[0]),
              sorted(set(acc[0]) ^ set(acc[1])), [])
        for n in ('RP2040_I2C_FIFO_DEPTH', 'RP2040_I2C_RX_BATCH', 'I2C_DW_RP2040_DMA_MAX_READ',
                  'IC_DATA_CMD_CMD', 'IC_DATA_CMD_STOP', 'IC_DATA_CMD_RESTART',
                  'DW_IC_REG_DATA_CMD', 'DW_IC_SPKLEN_MIN'):
            ck.eq(n, zh.get(n, (None, None))[1], ch.get(n, (None, None))[1])

        # The DMA path's events.
        zr = {n: v for n, v in defines(zc).items() if n.startswith('RP_DMA_')}
        cr = {n: v for n, v in defines(core_c).items() if n.startswith('RP_DMA_')}
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
              [ch['NANO_I2C_DW_FS_SPK_LEN'][1], ch['NANO_I2C_DW_HS_SPK_LEN'][1]])
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
                  int(ch['NANO_I2C_DW_TRANSFER_TIMEOUT_MS'][1]))
            ck.eq('clock-frequency of i2c0', one(r'clock-frequency = <\s*(\w+)\s*>', node, 'clock')[0],
                  '0x186a0')
        else:
            print('skip i2c_dw_sync .config and zephyr.dts (no build)')

        # The DMA path's values.
        zd = ' '.join(function(zc, 'i2c_dw_rp_dma_read').split())
        cd = ' '.join(function(core_c, 'i2c_dw_rp_dma_read').split())
        ck.eq('DMA lower limit', one(r'msg->len < (\d+)', zd, 'limit'),
              one(r'\blen < (\d+)', cd, 'limit'))
        ck.eq('DMA upper limit', one(r'msg->len > (\w+)', zd, 'limit'),
              one(r'\blen > (\w+)', cd, 'limit'))
        ck.eq('FIFO thresholds during DMA (value, register)',
              one(r'sys_write32\((\d+), base \+ (0x\w+)\)', zd, 'thresholds'),
              one(r'nano_dw_write32\((\d+), base \+ (0x\w+)\)', cd, 'thresholds'))

        def mask(pattern, text, what):
            return sorted(re.findall(r'DW_INTR_STAT_\w+', one(pattern, text, what)[0]))
        ck.eq('interrupt mask during DMA',
              mask(r'write_intr_mask\((DW_INTR_STAT[^;]*), base\)', zd, 'mask'),
              mask(r'write_intr_mask\((DW_INTR_STAT[^;]*), base\)', cd, 'mask'))
        zi = ' '.join(function(zc, 'i2c_dw_isr').split())
        ci = ' '.join(function(core_c, 'i2c_dw_isr_handler').split())
        ck.eq('errors of the DMA interrupt',
              mask(r'rp_dma_active\)\) \{ atomic_val_t events = 0; if \(intr_stat\.raw & '
                   r'\(([^)]*)\)', zi, 'isr'),
              mask(r'rp_dma_active\) \{ uint32_t events = 0; if \(intr_stat\.raw & '
                   r'\(([^)]*)\)', ci, 'isr'))
        ck.eq('DMA waits (ms)', one(r'k_uptime_get\(\) \+ (\d+)', zc, 'waits'),
              one(r'rp_dma_uptime_ms\(\) \+ (\d+)', core_c, 'waits'))
        ck.eq('DMA timeout', one(r'sys_timepoint_calc\((rom->transfer_timeout)\)', zd, 'timeout'),
              [x.replace('NANO_I2C_DW_TRANSFER_TIMEOUT_MS', 'rom->transfer_timeout') for x in
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
