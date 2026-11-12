# SPDX-License-Identifier: Apache-2.0
"""記録から imu_tk で加速度・角速度の校正値を求める。

imu_tk は https://github.com/owhinata/ubuntu-imu_tk の Docker イメージで実行する。
校正のパラメーターはイメージのビルド時に決まるので、記録の ODR と感度に合わせた
イメージがなければビルドする。
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

from . import recording

IMU_TK_REPO = 'https://github.com/owhinata/ubuntu-imu_tk.git'
# 完全なハッシュで指定する（docker build は省略形を取得できない）。
IMU_TK_COMMIT = '6143cdfab14514c6c579013cfc646d36808e8fbb'
G_STANDARD = 9.80665
SATURATION = 32760
# 姿勢を変えるときの回転のうち、校正の情報になる分（重力の向きを変える分）の軸ごとの
# 合計 [度]。目安に届かない軸は角速度のスケールが決まりにくく、不足を下回ると
# 結果を信頼できない。
ROTATION_TARGET_DEG = 1000.0
ROTATION_MIN_DEG = 500.0
MOVING_DPS = 2.0


def informative_rotation(gravity, rate_dps, seconds):
    """重力の向きを変える回転の量を軸ごとに返す [度]。

    gravity は機体座標での重力の向き（大きさは任意）、rate_dps は角速度。ある軸まわりの
    回転は、その軸が重力と直交しているほど重力の向きを大きく変える。重力と同じ軸
    まわりの回転は、加速度から見て姿勢が変わらず校正の情報にならない。そこで各軸の
    回転に sin（軸と重力のなす角）を掛けて数える。配列（N×3）でも 1 件（3）でもよい。
    """
    g = gravity / np.linalg.norm(gravity, axis=-1, keepdims=True)
    return np.abs(rate_dps) * np.sqrt(np.clip(1.0 - g * g, 0.0, 1.0)) * seconds


def rotation_coverage(rec, acc, gyro, window=0.1):
    """記録全体について、情報になる回転の軸ごとの合計 [度] を返す。"""
    n = int(float(rec.info['gyro_odr_hz']) * window)
    count = min(len(rec.gyro), len(rec.accel)) // n
    g = apply(gyro, rec.gyro[:count * n].astype(np.float64).reshape(count, n, 3).mean(axis=1))
    a = apply(acc, rec.accel[:count * n].astype(np.float64).reshape(count, n, 3).mean(axis=1))
    rate = g * 180.0 / math.pi
    moving = np.linalg.norm(rate, axis=1) >= MOVING_DPS
    seconds = n * rec.seconds(rec.gyro_tick[-1] - rec.gyro_tick[0]) / (len(rec.gyro) - 1)
    return informative_rotation(a[moving], rate[moving], seconds).sum(axis=0)


def _odd(n):
    return n if n % 2 else n + 1


def parameters(rec, args):
    """記録の設定から imu_tk のビルド引数を決める。"""
    odr = float(rec.info['accel_odr_hz'])
    samples = int(round(odr * args.interval))
    gyro_lsb = 1.0 / (rec.info['gyro_udps_per_lsb'] * 1e-6 * math.pi / 180.0)
    return dict(GRAVITY='%.4f' % args.gravity, INIT_STATIC='%.1f' % args.init_static,
                INTERVAL_SAMPLES=str(samples), WIN_SIZE=str(_odd(samples)),
                GYRO_LSB_PER_RAD_S='%.1f' % gyro_lsb)


def image_for(params):
    tag = 'imu-host-imu_tk:%s-g%s-s%s-i%s-w%s-k%s' % (
        IMU_TK_COMMIT[:7], params['GRAVITY'], params['INIT_STATIC'], params['INTERVAL_SAMPLES'],
        params['WIN_SIZE'], params['GYRO_LSB_PER_RAD_S'])
    if subprocess.run(['docker', 'image', 'inspect', tag], capture_output=True).returncode == 0:
        return tag
    print('imu_tk のイメージをビルドします（初回は 10 分以上かかります）: %s' % tag)
    cmd = ['docker', 'build', '--network', 'host', '-t', tag]
    for k, v in params.items():
        cmd += ['--build-arg', '%s=%s' % (k, v)]
    cmd.append('%s#%s' % (IMU_TK_REPO, IMU_TK_COMMIT))
    if subprocess.run(cmd).returncode:
        raise RuntimeError('imu_tk のイメージをビルドできませんでした')
    return tag


def check(rec, args):
    """校正に使えない理由の一覧を返す。"""
    problems = list(rec.problems)
    for name, data in (('加速度', rec.accel), ('角速度', rec.gyro)):
        n = int((np.abs(data.astype(np.int32)) >= SATURATION).any(axis=1).sum())
        if n:
            problems.append('%s が %d サンプル飽和' % (name, n))
    seconds = rec.seconds(rec.accel_tick[-1] - rec.accel_tick[0])
    if seconds < args.init_static + 30:
        problems.append('記録が短すぎます（%.0f 秒）' % seconds)
    return problems


def write_inputs(rec, work):
    """imu_tk の入力（時刻[秒] x y z、LSB のまま）を書く。"""
    for name, ticks, data in (('acc.txt', rec.accel_tick, rec.accel),
                              ('gyro.txt', rec.gyro_tick, rec.gyro)):
        table = np.column_stack((rec.seconds(ticks), data.astype(np.float64)))
        np.savetxt(os.path.join(work, name), table, fmt='%.7f %d %d %d')


def run_imu_tk(image, work):
    """imu_tk を実行する。出力は imu_tk.log に残し、要点だけ表示する。"""
    for name in ('test_imu_acc.calib', 'test_imu_gyro.calib'):
        if os.path.exists(os.path.join(work, name)):
            os.remove(os.path.join(work, name))
    cmd = ['docker', 'run', '-i', '--rm', '-u', '%d:%d' % (os.getuid(), os.getgid()),
           '-v', '%s:%s' % (work, work), '-w', work, image,
           'test_imu_calib', 'acc.txt', 'gyro.txt']
    started = time.monotonic()
    keys = ('Accelerometers calibration', 'Gyroscopes calibration', 'Termination',
            'residual', 'importAsciiData')
    text = []
    # test_imu_calib は加速度の計算後に Enter を待つので、改行を流しておく。
    with open(os.path.join(work, 'imu_tk.log'), 'w') as log:
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, errors='replace')
        p.stdin.write('\n' * 16)
        p.stdin.flush()
        for line in p.stdout:
            log.write(line)
            text.append(line)
            if any(k in line for k in keys) and 'Press Enter' not in line:
                print('  [%4.0f 秒] %s' % (time.monotonic() - started, line.rstrip()))
        p.stdin.close()
        p.wait()
    return ''.join(text), time.monotonic() - started


def read_calib(path):
    """imu_tk の .calib（ミスアライメント 3×3、スケール 3×3、バイアス 3）を読む。"""
    with open(path) as f:
        v = [float(x) for x in f.read().split()]
    if len(v) != 21 or not all(math.isfinite(x) for x in v):
        raise ValueError('%s の形式が想定と違います' % path)
    return dict(misalignment=np.array(v[:9]).reshape(3, 3), scale=np.array(v[9:18]).reshape(3, 3),
                bias=np.array(v[18:]))


def apply(calib, raw):
    """校正値を RAW（N×3）に適用する: T·K·(raw − bias)。"""
    return (raw - calib['bias']) @ (calib['misalignment'] @ calib['scale']).T


def static_windows(rec, seconds=1.0):
    """角速度のばらつきが小さい窓を静止とみなし、その窓ごとの平均 RAW を返す。

    検証用の単純な判定で、imu_tk が校正に使った区間とは一致しない。
    """
    odr = float(rec.info['gyro_odr_hz'])
    n = int(odr * seconds)
    count = min(len(rec.gyro), len(rec.accel)) // n
    g = rec.gyro[:count * n].astype(np.float64).reshape(count, n, 3)
    a = rec.accel[:count * n].astype(np.float64).reshape(count, n, 3)
    dps = rec.info['gyro_udps_per_lsb'] * 1e-6
    g_mean = g.mean(axis=1)
    spread = g.std(axis=1).max(axis=1) * dps
    # 窓の前半と後半の平均の差で、ゆっくりした回転を除く。
    drift = np.abs(g[:, :n // 2].mean(axis=1) - g[:, n // 2:].mean(axis=1)).max(axis=1) * dps
    a_drift = np.abs(a[:, :n // 2].mean(axis=1) - a[:, n // 2:].mean(axis=1)).max(axis=1)
    still = (spread < max(1.5 * np.percentile(spread, 10), 0.05)) & (drift < 0.2) & \
            (a_drift * rec.info['accel_ug_per_lsb'] * 1e-6 < 0.005)
    return a.mean(axis=1)[still], g_mean[still]


def validate(rec, acc, gyro, gravity):
    a_raw, g_raw = static_windows(rec)
    out = dict(static_windows=int(len(a_raw)))
    if not len(a_raw):
        return out
    nominal_a = rec.info['accel_ug_per_lsb'] * 1e-6 * G_STANDARD
    nominal_g = rec.info['gyro_udps_per_lsb'] * 1e-6 * math.pi / 180.0

    def stats(v):
        return dict(mean=float(np.mean(v)), rms=float(np.sqrt(np.mean(np.square(v)))),
                    max_abs=float(np.max(np.abs(v))))

    out['accel_norm_error_mg'] = dict(
        nominal=stats((np.linalg.norm(a_raw * nominal_a, axis=1) - gravity) / G_STANDARD * 1e3),
        calibrated=stats((np.linalg.norm(apply(acc, a_raw), axis=1) - gravity) / G_STANDARD * 1e3))
    out['gyro_static_dps'] = dict(
        nominal=stats(np.linalg.norm(g_raw * nominal_g, axis=1) * 180 / math.pi),
        calibrated=stats(np.linalg.norm(apply(gyro, g_raw), axis=1) * 180 / math.pi))
    out['accel_scale_vs_nominal_percent'] = [
        float((acc['scale'][i, i] / nominal_a - 1) * 100) for i in range(3)]
    out['gyro_scale_vs_nominal_percent'] = [
        float((gyro['scale'][i, i] / nominal_g - 1) * 100) for i in range(3)]
    rotation = rotation_coverage(rec, acc, gyro)
    out['rotation_deg'] = [float(x) for x in rotation]
    out['warnings'] = [
        '%s 軸まわりの回転が不足（%.0f 度、目安 %.0f 度以上）。この軸の角速度のスケールと'
        'ミスアライメントは信頼できません' % ('XYZ'[i], rotation[i], ROTATION_TARGET_DEG)
        for i in range(3) if rotation[i] < ROTATION_MIN_DEG]
    return out


def _calib_json(c, unit):
    return dict(unit=unit, misalignment=c['misalignment'].tolist(), scale=c['scale'].tolist(),
                bias_lsb=c['bias'].tolist())


def run(args):
    path = os.path.abspath(args.recording)
    print('記録を読み込み中: %s' % path)
    rec = recording.load(path)
    seconds = rec.seconds(rec.accel_tick[-1] - rec.accel_tick[0])
    print('  セッション %s、%d バッチ、%.1f 秒、加速度 %d・角速度 %d サンプル'
          % (rec.session, rec.batches, seconds, len(rec.accel), len(rec.gyro)))
    print('  tick の速さ: %.6f µs（%s）' % (rec.seconds_per_tick * 1e6, rec.tick_rate_source))
    for w in rec.warnings:
        print('  注意: %s' % w)
    problems = check(rec, args)
    if problems:
        print('この記録は校正に使えません:', file=sys.stderr)
        for p in problems:
            print('  - %s' % p, file=sys.stderr)
        if not args.force:
            return 1
        print('--force が指定されたので続行します', file=sys.stderr)
    params = parameters(rec, args)
    try:
        image = image_for(params)
    except (RuntimeError, OSError) as e:
        print(e, file=sys.stderr)
        return 1
    work = os.path.join(path, 'calib')
    os.makedirs(work, exist_ok=True)
    print('imu_tk の入力を書き出し中…')
    write_inputs(rec, work)
    print('imu_tk を実行中（静止判定に数分かかります）: %s' % image)
    output, elapsed = run_imu_tk(image, work)
    acc_path = os.path.join(work, 'test_imu_acc.calib')
    gyro_path = os.path.join(work, 'test_imu_gyro.calib')
    if not (os.path.exists(acc_path) and os.path.exists(gyro_path)) or \
       "Can't obtain" in output:
        print('imu_tk が校正値を出せませんでした。静止した姿勢が 12 個以上あるか確認してください。'
              'ログ: %s' % os.path.join(work, 'imu_tk.log'), file=sys.stderr)
        return 1
    acc, gyro = read_calib(acc_path), read_calib(gyro_path)
    converged = 'Termination:' in output and 'NO_CONVERGENCE' not in output
    result = dict(
        version=1, created=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        formula='calibrated = misalignment @ scale @ (raw_lsb - bias_lsb)',
        device=rec.info.get('device'), chip_id=rec.session[:16], session=rec.session,
        sensor=rec.info.get('sensor'),
        accel_odr_hz=rec.info.get('accel_odr_hz'), gyro_odr_hz=rec.info.get('gyro_odr_hz'),
        accel_ug_per_lsb=rec.info.get('accel_ug_per_lsb'),
        gyro_udps_per_lsb=rec.info.get('gyro_udps_per_lsb'),
        accel=_calib_json(acc, 'm/s^2'), gyro=_calib_json(gyro, 'rad/s'),
        imu_tk=dict(repository=IMU_TK_REPO, commit=IMU_TK_COMMIT, image=image, parameters=params,
                    gyro_converged=converged, seconds=round(elapsed, 1)),
        recording=dict(path=path, seconds=round(seconds, 1), batches=rec.batches,
                       seconds_per_tick=rec.seconds_per_tick,
                       tick_rate_source=rec.tick_rate_source, problems=problems),
        validation=validate(rec, acc, gyro, args.gravity))
    out = os.path.join(work, 'calibration.json')
    with open(out, 'w') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    report(result)
    print('結果: %s' % out)
    if not converged:
        print('注意: 角速度の最適化が収束していません。結果は信頼できません', file=sys.stderr)
        return 1
    if result['validation'].get('warnings'):
        return 1
    return 0


def report(result):
    v = result['validation']
    print()
    print('校正結果（imu_tk %.0f 秒、角速度の収束: %s）'
          % (result['imu_tk']['seconds'], 'あり' if result['imu_tk']['gyro_converged'] else 'なし'))
    for name in ('accel', 'gyro'):
        c = result[name]
        print('  %s [%s]' % (name, c['unit']))
        print('    scale（対角）: %s' % ', '.join('%.6e' % c['scale'][i][i] for i in range(3)))
        print('    bias [LSB]  : %s' % ', '.join('%.2f' % b for b in c['bias_lsb']))
        print('    misalignment: %s' % ' / '.join(
            ' '.join('% .5f' % x for x in row) for row in c['misalignment']))
    if 'accel_norm_error_mg' not in v:
        print('  検証に使える静止区間がありませんでした')
        return
    print('  検証（静止していた 1 秒の窓 %d 個）' % v['static_windows'])
    print('    公称感度からのスケールのずれ [%%]: 加速度 %s / 角速度 %s' % (
        ', '.join('%+.2f' % x for x in v['accel_scale_vs_nominal_percent']),
        ', '.join('%+.2f' % x for x in v['gyro_scale_vs_nominal_percent'])))
    for key, label, unit in (('accel_norm_error_mg', '加速度の大きさと重力の差', 'mg'),
                             ('gyro_static_dps', '静止時の角速度の大きさ', 'dps')):
        for which, jp in (('nominal', '校正前'), ('calibrated', '校正後')):
            s = v[key][which]
            print('    %s（%s）: 平均 %+.3f、RMS %.3f、最大 %.3f %s'
                  % (label, jp, s['mean'], s['rms'], s['max_abs'], unit))
    print('    重力の向きを変えた回転の合計 [度]: X %.0f、Y %.0f、Z %.0f（目安 各 %.0f 以上）'
          % (*v['rotation_deg'], ROTATION_TARGET_DEG))
    for w in v['warnings']:
        print('  注意: %s' % w)
