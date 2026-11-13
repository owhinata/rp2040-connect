// SPDX-License-Identifier: Apache-2.0
using System.Text.Json;

namespace ImuStudio.App.Services;

/// <summary>What the application remembers between runs.</summary>
public sealed class AppSettings
{
    public int MqttPort { get; set; } = 1883;
    public string BoardAddress { get; set; } = "";
    public float Beta { get; set; } = 0.05f;
    public string RecordingsDirectory { get; set; } = "";
    public double ShellPaneHeight { get; set; } = 190;
    public bool ShellPaneVisible { get; set; } = true;
    /// <summary>The velocity model; empty for the place where tools/export_tartanimu.py puts it.</summary>
    public string PositionModelPath { get; set; } = "";
    public int SpectrumSize { get; set; } = 4096;
    public string SpectrumMaxFrequency { get; set; } = "Nyquist";
    public bool SpectrumLogFrequency { get; set; }
    public double SpectrumMinDb { get; set; } = -70;
    public double SpectrumMaxDb { get; set; } = 20;

    private static string FilePath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "ImuStudio", "settings.json");

    public static AppSettings Load()
    {
        try
        {
            return JsonSerializer.Deserialize<AppSettings>(File.ReadAllText(FilePath)) ?? new AppSettings();
        }
        catch (Exception e) when (e is IOException or JsonException or UnauthorizedAccessException)
        {
            return new AppSettings();
        }
    }

    public void Save()
    {
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);
            File.WriteAllText(FilePath, JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
        }
    }

    /// <summary>
    /// The velocity model: the configured file, or <c>models/</c> found from
    /// the application's own directory upwards. For a build inside the
    /// sources that is <c>host/ImuStudio/models/</c> (ignored by Git; where
    /// <c>tools/export_tartanimu.py</c> writes); for a copy of the application
    /// elsewhere, a <c>models/</c> directory beside it. The user's home is
    /// not searched. When there is no model, the answer is where to put it.
    /// </summary>
    public string ResolvePositionModelPath()
    {
        const string name = "tartanimu-human.onnx";
        if (!string.IsNullOrWhiteSpace(PositionModelPath))
        {
            return PositionModelPath;
        }
        for (DirectoryInfo? dir = new(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            string candidate = Path.Combine(dir.FullName, "models", name);
            if (File.Exists(candidate))
            {
                return candidate;
            }
        }
        string home = SourceDirectory()?.FullName ?? AppContext.BaseDirectory;
        return Path.Combine(home, "models", name);
    }

    /// <summary>
    /// Where recordings go: the configured directory, or <c>host/recordings</c>
    /// next to this application's sources (shared with <c>imu-host record</c>),
    /// or a directory in the user's home when run from elsewhere.
    /// </summary>
    public string ResolveRecordingsDirectory()
    {
        if (!string.IsNullOrWhiteSpace(RecordingsDirectory))
        {
            return RecordingsDirectory;
        }
        return SourceDirectory()?.Parent is DirectoryInfo host
            ? Path.Combine(host.FullName, "recordings")
            : Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "ImuStudio", "recordings");
    }

    /// <summary>The directory of this application's sources, if it runs from a build inside it.</summary>
    private static DirectoryInfo? SourceDirectory()
    {
        for (DirectoryInfo? dir = new(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            if (File.Exists(Path.Combine(dir.FullName, "ImuStudio.slnx")))
            {
                return dir;
            }
        }
        return null;
    }
}
