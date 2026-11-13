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
        for (DirectoryInfo? dir = new(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            if (File.Exists(Path.Combine(dir.FullName, "ImuStudio.slnx")) && dir.Parent is not null)
            {
                return Path.Combine(dir.Parent.FullName, "recordings");
            }
        }
        return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "ImuStudio", "recordings");
    }
}
