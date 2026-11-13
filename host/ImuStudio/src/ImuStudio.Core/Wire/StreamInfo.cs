// SPDX-License-Identifier: Apache-2.0
using System.Text.Json;

namespace ImuStudio.Core.Wire;

/// <summary>What the board says about a run in its <c>info</c> message.</summary>
public sealed record StreamInfo(
    string Session,
    int AccelOdrHz,
    int GyroOdrHz,
    uint AccelMicroGPerLsb,
    uint GyroMicroDpsPerLsb,
    int TickHzNominal,
    int FreqFine)
{
    public static StreamInfo? TryParse(ReadOnlySpan<byte> json)
    {
        try
        {
            using JsonDocument doc = JsonDocument.Parse(json.ToArray());
            JsonElement r = doc.RootElement;
            return new StreamInfo(
                r.GetProperty("session").GetString() ?? "",
                r.GetProperty("accel_odr_hz").GetInt32(),
                r.GetProperty("gyro_odr_hz").GetInt32(),
                r.GetProperty("accel_ug_per_lsb").GetUInt32(),
                r.GetProperty("gyro_udps_per_lsb").GetUInt32(),
                r.TryGetProperty("tick_hz_nominal", out JsonElement t) ? t.GetInt32() : 40000,
                r.TryGetProperty("freq_fine", out JsonElement f) ? f.GetInt32() : 0);
        }
        catch (Exception e) when (e is JsonException or KeyNotFoundException or InvalidOperationException or FormatException)
        {
            return null;
        }
    }
}
