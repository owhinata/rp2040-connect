// SPDX-License-Identifier: Apache-2.0
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace ImuStudio.Core.Processing;

/// <summary>
/// A learned estimate of the sensor's velocity from a stretch of IMU data.
/// The layout is that of the Tartan IMU model: one-second windows of
/// <see cref="WindowSamples"/> samples at 40 Hz, six rows each (gyro x, y, z in
/// rad/s, then acceleration x, y, z in m/s² with gravity), in the sensor's frame.
/// </summary>
public interface IVelocityModel : IDisposable
{
    public const int WindowSamples = 40;
    public const int Rows = 6;
    public const int MaxWindows = 10;

    /// <summary>
    /// The mean velocity over the last window, in m/s in the sensor's frame.
    /// </summary>
    /// <param name="imu">windows × 6 × 40 values: oldest window first, then row, then time.</param>
    System.Numerics.Vector3 Predict(ReadOnlySpan<float> imu, int windows);
}

/// <summary>The model as converted by <c>tools/export_tartanimu.py</c>.</summary>
public sealed class OnnxVelocityModel : IVelocityModel
{
    private readonly InferenceSession _session;
    private readonly string _input, _output;

    public OnnxVelocityModel(string path)
    {
        // One thread, and no busy waiting between runs: the model is small and
        // runs sixty times a second, so idle threads spinning would cost more
        // processor time than the model itself.
        SessionOptions options = new() { IntraOpNumThreads = 1, InterOpNumThreads = 1 };
        options.AddSessionConfigEntry("session.intra_op.allow_spinning", "0");
        _session = new InferenceSession(path, options);
        _input = _session.InputMetadata.Keys.First();
        _output = _session.OutputMetadata.Keys.First();
    }

    public System.Numerics.Vector3 Predict(ReadOnlySpan<float> imu, int windows)
    {
        DenseTensor<float> tensor = new(imu.ToArray(), [1, windows, IVelocityModel.Rows, IVelocityModel.WindowSamples]);
        using IDisposableReadOnlyCollection<DisposableNamedOnnxValue> results =
            _session.Run([NamedOnnxValue.CreateFromTensor(_input, tensor)], [_output]);
        Tensor<float> velocity = results[0].AsTensor<float>();
        int last = windows - 1;
        return new System.Numerics.Vector3(velocity[0, last, 0], velocity[0, last, 1], velocity[0, last, 2]);
    }

    public void Dispose() => _session.Dispose();
}
