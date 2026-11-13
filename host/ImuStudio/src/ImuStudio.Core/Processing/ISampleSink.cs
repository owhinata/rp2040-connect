// SPDX-License-Identifier: Apache-2.0
using System.Numerics;

namespace ImuStudio.Core.Processing;

/// <summary>
/// What consumes the converted samples. The pipeline feeds exactly one sink,
/// the one of the view in use.
/// </summary>
public interface ISampleSink
{
    /// <summary>A new run starts: nothing of what came before applies.</summary>
    void Reset();

    /// <summary>Samples of the run were lost; the next ones are later than the period says.</summary>
    void OnGap();

    /// <param name="metersPerSecond2">Specific force in the sensor frame.</param>
    void OnAccel(Vector3 metersPerSecond2);

    /// <param name="radiansPerSecond">Angular rate in the sensor frame.</param>
    /// <param name="dt">Seconds since the previous gyro sample.</param>
    void OnGyro(Vector3 radiansPerSecond, double dt);
}
