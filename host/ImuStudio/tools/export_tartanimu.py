#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Converts the released Tartan IMU model to ONNX for ImuStudio.

Tartan IMU (https://github.com/superxslam/TartanIMU, code Apache-2.0, weights
MIT) estimates the velocity in the sensor's frame from 6-axis IMU data. This
script downloads the released weights, keeps the head for human motion and
writes one ONNX file. The model is not committed to this repository: the
output goes to the models directory next to this one, which Git ignores and
where ImuStudio looks for it.

    git clone https://github.com/superxslam/TartanIMU.git /path/to/TartanIMU
    git -C /path/to/TartanIMU checkout 8a8025000f85d2c6b0692b88e15c2c91e761f7d7
    python3 -m venv /path/to/venv
    /path/to/venv/bin/pip install --index-url https://download.pytorch.org/whl/cpu torch
    /path/to/venv/bin/pip install onnx onnxruntime onnxscript pyyaml numpy einops
    /path/to/venv/bin/python tools/export_tartanimu.py --source /path/to/TartanIMU

Input  "imu":      float32 [1, windows, 6, 40]. Each window is one second at
                   40 Hz; rows are gyro x, y, z in rad/s, then acceleration
                   x, y, z in m/s^2 with gravity, in the sensor's frame.
Output "velocity": float32 [1, windows, 3]. Mean velocity over each window
                   in m/s, in the sensor's frame.
"""
import argparse
import hashlib
import os
import sys
import urllib.request

WEIGHTS_URL = 'https://huggingface.co/Tartan-IMU/TartanIMU/resolve/main/'
FILES = ('checkpoints/unified.pt', 'config/unified.yaml', 'config/resnet_lstm_multihead.yaml')
HEADS = ('car', 'dog', 'drone', 'human')
DEFAULT_OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')


def download(directory):
    for name in FILES:
        path = os.path.join(directory, name)
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            print('downloading', name)
            urllib.request.urlretrieve(WEIGHTS_URL + name, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', required=True, help='checkout of superxslam/TartanIMU')
    parser.add_argument('--weights', help='directory for the downloaded weights (default: <out>/tartanimu-weights)')
    parser.add_argument('--out', default=DEFAULT_OUT, help='output directory (default: %(default)s)')
    parser.add_argument('--head', default='human', choices=HEADS)
    args = parser.parse_args()

    import numpy as np
    import onnxruntime
    import torch

    out_dir = os.path.abspath(args.out)
    weights = os.path.abspath(args.weights or os.path.join(out_dir, 'tartanimu-weights'))
    download(weights)

    # The model is built through the registry, not through the repository's
    # configer: that one imports the training code and its experiment tracker.
    import yaml
    sys.path.insert(0, os.path.abspath(args.source))
    from tartan_imu.model.registry import build_backbone

    def merge(into, other):
        for key, value in other.items():
            if isinstance(value, dict):
                merge(into.setdefault(key, {}), value)
            else:
                into[key] = value

    with open(os.path.join(weights, 'config', 'unified.yaml')) as f:
        config = yaml.safe_load(f)
    # The experiment YAML names the model YAML relative to the weights directory.
    with open(os.path.join(weights, config['model']['model_yaml'])) as f:
        merge(config, yaml.safe_load(f))
    model = build_backbone(config['model']['model_name'], config)
    checkpoint_path = os.path.join(weights, 'checkpoints', 'unified.pt')
    # Besides tensors the checkpoint holds plain numpy numbers. Only those are
    # let through; the file is never loaded in the mode that can run code.
    harmless = {'numpy.dtype', 'numpy.core.multiarray.scalar', 'numpy._core.multiarray.scalar'}
    needed = set(torch.serialization.get_unsafe_globals_in_checkpoint(checkpoint_path))
    if needed - harmless:
        sys.exit('The checkpoint refers to unexpected types: %s' % ', '.join(sorted(needed - harmless)))
    # numpy 2 moved "core" to "_core"; the checkpoint may name either.
    scalar = getattr(np, '_core', None) or np.core
    scalar = scalar.multiarray.scalar
    numbers = [np.dtype, (scalar, 'numpy.core.multiarray.scalar'), (scalar, 'numpy._core.multiarray.scalar')] + \
        [type(np.dtype(t)) for t in (np.float64, np.float32, np.int64, np.int32, np.bool_)]
    with torch.serialization.safe_globals(numbers):
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True)
    model.load_state_dict(checkpoint.get('model_state_dict', checkpoint))
    model.eval()

    samples = int(config['data']['sample_freq'] * config['model_param']['window_time'])
    label = torch.tensor([HEADS.index(args.head) + 1])

    class OneHead(torch.nn.Module):
        def __init__(self, inner):
            super().__init__()
            self.inner = inner

        def forward(self, imu):
            out = self.inner(imu, motion_type=label, predict_cov=False, compute_all_heads=False)
            return out[args.head]

    wrapped = OneHead(model).eval()
    example = torch.randn(1, config['train']['seq_len'], 6, samples)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'tartanimu-%s.onnx' % args.head)
    torch.onnx.export(wrapped, (example,), out_path, input_names=['imu'], output_names=['velocity'],
                      dynamic_axes={'imu': {1: 'windows'}, 'velocity': {1: 'windows'}},
                      opset_version=17, dynamo=False)

    # The converted model must give what the original gives, also for fewer windows.
    session = onnxruntime.InferenceSession(out_path, providers=['CPUExecutionProvider'])
    worst = 0.0
    for windows in (config['train']['seq_len'], 3, 1):
        x = torch.randn(1, windows, 6, samples)
        x[:, :, 5, :] += 9.81
        with torch.no_grad():
            expected = wrapped(x).numpy()
        got = session.run(None, {'imu': x.numpy()})[0]
        assert got.shape == (1, windows, 3), got.shape
        worst = max(worst, float(np.abs(got - expected).max()))
    if worst > 1e-3:
        sys.exit('The ONNX model differs from the original by %.2e m/s' % worst)

    with open(checkpoint_path, 'rb') as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    print('wrote %s (%d bytes)' % (out_path, os.path.getsize(out_path)))
    print('head %s, %d samples per window, largest difference from the original %.1e m/s' % (args.head, samples, worst))
    print('weights sha256 %s' % digest)


if __name__ == '__main__':
    main()
