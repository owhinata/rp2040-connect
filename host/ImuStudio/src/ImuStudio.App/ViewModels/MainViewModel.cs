// SPDX-License-Identifier: Apache-2.0
using System.Globalization;
using System.Numerics;
using System.Text;
using System.Text.RegularExpressions;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using ImuStudio.App.Services;
using ImuStudio.Core.Board;
using ImuStudio.Core.Calibration;
using ImuStudio.Core.Mqtt;
using ImuStudio.Core.Processing;
using ImuStudio.Core.Recording;
using ImuStudio.Core.Wire;

namespace ImuStudio.App.ViewModels;

public sealed partial class MainViewModel : ObservableObject, IDisposable
{
    private const int ShellLogLimit = 40_000;
    private static readonly TimeSpan CommandWait = TimeSpan.FromSeconds(2);

    private readonly AppSettings _settings = AppSettings.Load();
    private readonly ImuPipeline _pipeline = new();
    private readonly AttitudeEstimator _attitude = new();
    private readonly SpectrumAnalyzer _spectrum = new();
    private PositionEstimator? _position;
    private readonly DispatcherTimer _timer;
    private readonly StringBuilder _shellLog = new();
    private readonly HashSet<string> _calibrationRead = new();
    private readonly object _recordLock = new();
    private MqttEndpoint? _endpoint;
    private BoardShell? _shell;
    private RecordingWriter? _writer;
    private RecordingPlayer? _player;
    private (string Topic, byte[] Payload)? _lastInfo;
    private long _recordStartMissing;
    private volatile bool _playing;
    private bool _showingReplayPosition;
    private int _frame;

    [ObservableProperty] private string _mqttPort;
    [ObservableProperty] private string _listenStatus = "";
    [ObservableProperty] private string _boardAddress;
    [ObservableProperty] private bool _isShellConnected;
    [ObservableProperty] private string _shellText = "";
    [ObservableProperty] private string _shellInput = "";
    [ObservableProperty] private bool _isRecording;
    [ObservableProperty] private bool _isPlaying;
    [ObservableProperty] private string _message = "";
    [ObservableProperty] private string _streamStatus = "No data";
    [ObservableProperty] private string _attitudeText = "";
    [ObservableProperty] private Quaternion _orientation = Quaternion.Identity;
    [ObservableProperty] private double _beta;
    [ObservableProperty] private bool _isReplayPaused;
    [ObservableProperty] private double _replayPosition;
    [ObservableProperty] private double _replayDuration = 1;
    [ObservableProperty] private string _replayTime = "";
    [ObservableProperty] private bool _isShellPaneVisible;
    [ObservableProperty] private int _selectedView;
    [ObservableProperty] private int _spectrumSize;
    [ObservableProperty] private string _spectrumMaxFrequency;
    [ObservableProperty] private bool _spectrumLogFrequency;
    [ObservableProperty] private double _spectrumMinDb;
    [ObservableProperty] private double _spectrumMaxDb;
    [ObservableProperty] private string _spectrumText = "";
    [ObservableProperty] private string _positionText = "";

    public const int AttitudeView = 0;
    public const int SpectrumView = 1;
    public const int PositionView = 2;

    /// <summary>Raised on the UI thread at the frame rate: the estimate, and the path when it was copied anew.</summary>
    public event Action<PositionSnapshot, Vector3[]?>? PositionUpdated;

    public IReadOnlyList<int> SpectrumSizes { get; } = SpectrumAnalyzer.Sizes;
    public IReadOnlyList<string> SpectrumMaxFrequencies { get; } = ["Nyquist", "1600", "800", "400", "200", "100"];

    /// <summary>Raised on the UI thread, at the frame rate, with the spectra made since the last frame.</summary>
    public event Action<List<SpectrumColumn>>? SpectrumColumns;

    public List<string> ShellHistory { get; } = new();

    /// <param name="port">MQTT port to use instead of the remembered one.</param>
    /// <param name="board">Board address to connect the shell to at once.</param>
    public MainViewModel(string? port = null, string? board = null)
    {
        _mqttPort = port ?? _settings.MqttPort.ToString(CultureInfo.InvariantCulture);
        _boardAddress = board ?? _settings.BoardAddress;
        _beta = _settings.Beta;
        _attitude.Beta = _settings.Beta;
        _isShellPaneVisible = _settings.ShellPaneVisible;
        _spectrumSize = Array.IndexOf(SpectrumAnalyzer.Sizes, _settings.SpectrumSize) >= 0 ? _settings.SpectrumSize : 4096;
        _spectrumMaxFrequency = SpectrumMaxFrequencies.Contains(_settings.SpectrumMaxFrequency)
            ? _settings.SpectrumMaxFrequency : "Nyquist";
        _spectrumLogFrequency = _settings.SpectrumLogFrequency;
        _spectrumMinDb = _settings.SpectrumMinDb;
        _spectrumMaxDb = _settings.SpectrumMaxDb;
        _spectrum.Size = _spectrumSize;
        _pipeline.Sink = _attitude;
        _pipeline.InfoReceived += info => Dispatcher.UIThread.Post(() => _ = ReadCalibrationAsync(info));
        Listen();
        _timer = new DispatcherTimer(TimeSpan.FromMilliseconds(1000.0 / 60), DispatcherPriority.Render, (_, _) => Tick());
        _timer.Start();
        if (board is not null)
        {
            _ = ToggleShellAsync();
        }
    }

    partial void OnBetaChanged(double value)
    {
        _attitude.Beta = (float)value;
        if (_position is not null)
        {
            _position.Beta = (float)value;
        }
        _settings.Beta = (float)value;
    }

    partial void OnIsShellPaneVisibleChanged(bool value) => _settings.ShellPaneVisible = value;

    /// <summary>The views take turns: only the one shown gets the samples.</summary>
    partial void OnSelectedViewChanged(int value) =>
        _pipeline.Sink = value switch
        {
            SpectrumView => _spectrum,
            PositionView => LoadPositionModel(),
            _ => _attitude,
        };

    /// <summary>The estimator, made when the view is first opened. Null without the model.</summary>
    private PositionEstimator? LoadPositionModel()
    {
        if (_position is not null)
        {
            return _position;
        }
        string path = _settings.ResolvePositionModelPath();
        if (!File.Exists(path))
        {
            PositionText = $"The velocity model is missing:\n{path}\n\n" +
                           "Make it with tools/export_tartanimu.py\n(see the README), then open this\nview again.";
            return null;
        }
        try
        {
            _position = new PositionEstimator(new OnnxVelocityModel(path)) { Beta = (float)Beta };
        }
        catch (Exception e)
        {
            PositionText = $"Cannot load the velocity model:\n{path}\n\n{e.Message}";
        }
        return _position;
    }

    /// <summary>Back to the origin; the attitude is taken again from the acceleration.</summary>
    [RelayCommand]
    private void ResetPosition()
    {
        if (SelectedView == PositionView)
        {
            _pipeline.Sink = _position;
        }
    }

    partial void OnSpectrumSizeChanged(int value)
    {
        _spectrum.Size = value;
        _settings.SpectrumSize = value;
    }

    partial void OnSpectrumMaxFrequencyChanged(string value) => _settings.SpectrumMaxFrequency = value;
    partial void OnSpectrumLogFrequencyChanged(bool value) => _settings.SpectrumLogFrequency = value;
    partial void OnSpectrumMinDbChanged(double value) => _settings.SpectrumMinDb = value;
    partial void OnSpectrumMaxDbChanged(double value) => _settings.SpectrumMaxDb = value;

    /// <summary>Upper end of the frequency axis in Hz; infinity for all there is.</summary>
    public double SpectrumMaxHz =>
        double.TryParse(SpectrumMaxFrequency, NumberStyles.Float, CultureInfo.InvariantCulture, out double hz)
            ? hz : double.PositiveInfinity;

    /// <summary>Takes the direction of gravity again, from the next quarter second.</summary>
    [RelayCommand]
    private void LevelSpectrum()
    {
        if (SelectedView == SpectrumView)
        {
            _pipeline.Sink = _spectrum;
        }
    }

    /// <summary>Height of the shell pane in the window; kept between runs.</summary>
    public double ShellPaneHeight
    {
        get => _settings.ShellPaneHeight;
        set => _settings.ShellPaneHeight = value;
    }

    // ---- receiving -------------------------------------------------------

    [RelayCommand]
    private void Listen()
    {
        _endpoint?.Dispose();
        _endpoint = null;
        if (!int.TryParse(MqttPort, NumberStyles.None, CultureInfo.InvariantCulture, out int port) || port is < 1 or > 65535)
        {
            ListenStatus = "invalid port";
            return;
        }
        try
        {
            MqttEndpoint endpoint = new(port);
            endpoint.Published += OnLiveMessage;
            _endpoint = endpoint;
            _settings.MqttPort = port;
            ListenStatus = $"listening on {port}";
        }
        catch (System.Net.Sockets.SocketException e)
        {
            ListenStatus = $"cannot listen on {port}: {e.Message}";
        }
    }

    /// <summary>Runs on the receiving thread.</summary>
    private void OnLiveMessage(string topic, byte[] payload)
    {
        if (_playing)
        {
            return;
        }
        lock (_recordLock)
        {
            if (topic.EndsWith("/info", StringComparison.Ordinal))
            {
                _lastInfo = (topic, payload);
            }
            _writer?.Add(topic, payload);
        }
        _pipeline.OnMessage(topic, payload);
    }

    // ---- board shell -----------------------------------------------------

    [RelayCommand]
    private async Task FindBoardAsync()
    {
        try
        {
            BoardAddress = await BoardLocator.FindAsync();
            Message = $"Board found at {BoardAddress}";
        }
        catch (BoardException e)
        {
            Message = e.Message;
        }
    }

    [RelayCommand]
    private async Task ToggleShellAsync()
    {
        if (_shell is not null)
        {
            _shell.Dispose();
            return;
        }
        try
        {
            if (string.IsNullOrWhiteSpace(BoardAddress))
            {
                BoardAddress = await BoardLocator.FindAsync();
            }
            // "address" or "address:port"; the port is for a shell that is not on the Telnet port.
            string[] parts = BoardAddress.Trim().Split(':');
            int port = BoardLocator.TelnetPort;
            if (parts.Length > 2 || (parts.Length == 2 && (!int.TryParse(parts[1], out port) || port is < 1 or > 65535)))
            {
                Message = "The board address is not of the form address or address:port";
                return;
            }
            BoardShell shell = await BoardShell.ConnectAsync(parts[0], port);
            shell.Output += text => Dispatcher.UIThread.Post(() => AppendShell(text));
            shell.Closed += reason => Dispatcher.UIThread.Post(() => OnShellClosed(shell, reason));
            _shell = shell;
            _calibrationRead.Clear();
            IsShellConnected = true;
            _settings.BoardAddress = BoardAddress.Trim();
            Message = $"Shell connected to {shell.Address}";
            await shell.SettleAsync();
        }
        catch (BoardException e)
        {
            Message = e.Message;
        }
    }

    private void OnShellClosed(BoardShell shell, string? reason)
    {
        if (_shell != shell)
        {
            return;
        }
        _shell = null;
        IsShellConnected = false;
        Message = reason is null ? "Shell disconnected" : $"Shell disconnected: {reason}";
    }

    private void AppendShell(string text)
    {
        _shellLog.Append(text);
        if (_shellLog.Length > ShellLogLimit)
        {
            _shellLog.Remove(0, _shellLog.Length - ShellLogLimit * 3 / 4);
        }
        ShellText = _shellLog.ToString();
    }

    /// <summary>Runs a line on the board and shows it in the shell pane. Null when it could not be sent.</summary>
    private async Task<string?> RunAsync(string line)
    {
        BoardShell? shell = _shell;
        if (shell is null)
        {
            Message = "The shell is not connected";
            return null;
        }
        AppendShell(line);
        try
        {
            return await shell.ExecuteAsync(line, CommandWait);
        }
        catch (Exception e) when (e is BoardException or ObjectDisposedException)
        {
            Message = e.Message;
            return null;
        }
    }

    [RelayCommand]
    private async Task SendShellAsync()
    {
        string line = ShellInput.Trim();
        if (line.Length == 0)
        {
            return;
        }
        ShellInput = "";
        ShellHistory.Remove(line);
        ShellHistory.Add(line);
        await RunAsync(line);
    }

    [RelayCommand]
    private Task ImuStartAsync() => RunAsync("imu start");

    [RelayCommand]
    private Task ImuStopAsync() => RunAsync("imu stop");

    /// <summary>
    /// Reads the calibration for the announced sensitivities from the board's
    /// KV store, once per shell connection, and hands it to the pipeline.
    /// </summary>
    private async Task ReadCalibrationAsync(StreamInfo info)
    {
        if (_playing || _shell is null)
        {
            return;
        }
        foreach ((SensorKind kind, uint gain) in new[]
                 {
                     (SensorKind.Accel, info.AccelMicroGPerLsb), (SensorKind.Gyro, info.GyroMicroDpsPerLsb),
                 })
        {
            string key = SensorCalibration.Key(kind, gain);
            if (!_calibrationRead.Add(key))
            {
                continue;
            }
            string line = "kv get " + key;
            string? captured = await RunAsync(line);
            if (captured is null)
            {
                _calibrationRead.Remove(key);
                return;
            }
            string value = string.Concat(BoardShell.OutputLines(captured, line)).Replace(" ", "", StringComparison.Ordinal);
            byte[]? blob = HexValue().IsMatch(value) ? Convert.FromHexString(value) : null;
            if (blob is not null && SensorCalibration.TryDecode(blob, kind, gain) is null)
            {
                Message = $"{key} on the board is not a calibration for this sensitivity";
                blob = null;
            }
            _pipeline.SetStored(key, blob);
        }
    }

    [RelayCommand]
    private void ReloadCalibration()
    {
        _calibrationRead.Clear();
        if (_pipeline.Snapshot().Info is StreamInfo info)
        {
            _ = ReadCalibrationAsync(info);
        }
    }

    [GeneratedRegex("^([0-9a-fA-F]{2})+$")]
    private static partial Regex HexValue();

    // ---- recording and replay -------------------------------------------

    [RelayCommand]
    private void ToggleRecord()
    {
        if (_writer is not null)
        {
            StopRecording();
            return;
        }
        if (_playing)
        {
            Message = "Stop the replay before recording";
            return;
        }
        try
        {
            string path = Path.Combine(_settings.ResolveRecordingsDirectory(),
                                       DateTime.Now.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture));
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            RecordingWriter writer = new(path);
            lock (_recordLock)
            {
                // A recording that starts in the middle of a run still needs the run's info.
                if (_lastInfo is (string topic, byte[] payload))
                {
                    writer.Add(topic, payload);
                }
                _writer = writer;
            }
            _recordStartMissing = _pipeline.Snapshot().MissingBatches;
            IsRecording = true;
            Message = $"Recording to {path}";
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            Message = $"Cannot record: {e.Message}";
        }
    }

    private void StopRecording()
    {
        RecordingWriter? writer;
        lock (_recordLock)
        {
            writer = _writer;
            _writer = null;
        }
        if (writer is null)
        {
            return;
        }
        PipelineSnapshot s = _pipeline.Snapshot();
        Dictionary<string, object?> summary = new()
        {
            ["batches"] = writer.RawPayloads,
            ["missing_batches"] = s.MissingBatches - _recordStartMissing,
            ["session"] = s.Session,
            ["board"] = _endpoint?.Peer,
            ["port"] = _endpoint?.Port,
            ["recorder"] = "ImuStudio",
        };
        Dictionary<string, byte[]> calibration = new();
        if (s.Info is StreamInfo info)
        {
            foreach (string key in new[]
                     {
                         SensorCalibration.Key(SensorKind.Accel, info.AccelMicroGPerLsb),
                         SensorCalibration.Key(SensorKind.Gyro, info.GyroMicroDpsPerLsb),
                     })
            {
                if (_pipeline.Stored.TryGetValue(key, out byte[]? blob))
                {
                    calibration[key] = blob;
                }
            }
        }
        try
        {
            writer.Close(summary, calibration);
            Message = $"Recorded {writer.RawPayloads} batches to {writer.Path}";
        }
        catch (IOException e)
        {
            Message = $"Cannot finish the recording: {e.Message}";
        }
        IsRecording = false;
    }

    public string RecordingsDirectory => _settings.ResolveRecordingsDirectory();

    public void Play(string path)
    {
        StopReplay();
        if (_writer is not null)
        {
            StopRecording();
        }
        RecordingPlayer player;
        try
        {
            player = new RecordingPlayer(path);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            Message = e.Message;
            return;
        }
        _playing = true;
        _pipeline.Clear();
        foreach ((string key, byte[] blob) in player.Calibration)
        {
            _pipeline.SetStored(key, blob);
        }
        // After a move to another time, what follows is a new run for the pipeline.
        player.Restarted += _pipeline.Clear;
        player.Published += _pipeline.OnMessage;
        player.Finished += () => Dispatcher.UIThread.Post(() =>
        {
            if (_player == player)
            {
                Message = $"Replay of {Path.GetFileName(path)} finished";
                StopReplay();
            }
        });
        _player = player;
        IsReplayPaused = false;
        ReplayDuration = Math.Max(player.Duration, 0.001);
        ShowReplayPosition(0);
        IsPlaying = true;
        Message = $"Replaying {path}" + (player.Calibration.Count == 0 ? " (no calibration saved with it)" : "");
        player.Start();
    }

    [RelayCommand]
    private void StopReplay()
    {
        RecordingPlayer? player = _player;
        if (player is null)
        {
            return;
        }
        _player = null;
        player.Dispose();
        _playing = false;
        IsPlaying = false;
        // Back to the live stream: the board's info and calibration are asked for again.
        _pipeline.Clear();
        _calibrationRead.Clear();
        lock (_recordLock)
        {
            if (_lastInfo is (string topic, byte[] payload))
            {
                _pipeline.OnMessage(topic, payload);
            }
        }
    }

    [RelayCommand]
    private void ToggleReplayPause()
    {
        if (_player is RecordingPlayer player)
        {
            player.IsPaused = !player.IsPaused;
            IsReplayPaused = player.IsPaused;
        }
    }

    /// <summary>The slider was moved by hand: continue from there.</summary>
    partial void OnReplayPositionChanged(double value)
    {
        if (!_showingReplayPosition)
        {
            _player?.Seek(value);
        }
    }

    private void ShowReplayPosition(double seconds)
    {
        _showingReplayPosition = true;
        ReplayPosition = seconds;
        _showingReplayPosition = false;
        ReplayTime = string.Create(CultureInfo.InvariantCulture, $"{seconds,6:F1} / {ReplayDuration:F1} s");
    }

    // ---- display ---------------------------------------------------------

    [RelayCommand]
    private void ResetAttitude() => _pipeline.Sink = _attitude;

    private void Tick()
    {
        if (SelectedView == SpectrumView)
        {
            List<SpectrumColumn> columns = _spectrum.Drain();
            if (columns.Count > 0)
            {
                SpectrumColumns?.Invoke(columns);
                if (_frame % 6 == 0)
                {
                    ShowSpectrumText(columns[^1]);
                }
            }
        }
        else if (SelectedView == PositionView)
        {
            if (_position is not null)
            {
                PositionSnapshot p = _position.Snapshot();
                bool slow = _frame % 6 == 0;
                PositionUpdated?.Invoke(p, slow ? _position.Track() : null);
                if (slow)
                {
                    ShowPositionText(p);
                }
            }
        }
        else
        {
            Orientation = _attitude.Orientation;
        }
        if (++_frame % 6 != 0)
        {
            return;
        }
        PipelineSnapshot s = _pipeline.Snapshot();
        RecordingPlayer? player = _player;
        StringBuilder status = new();
        if (player is not null)
        {
            ShowReplayPosition(player.Position);
            status.Append(IsReplayPaused ? "Replay (paused)" : "Replay");
        }
        else
        {
            MqttEndpoint? endpoint = _endpoint;
            status.Append(endpoint is { HasClient: true } ? $"Live from {endpoint.Peer}" : "Live: no client");
            if (endpoint?.ProtocolError is string error)
            {
                status.Append(" (").Append(error).Append(')');
            }
        }
        if (s.Info is null)
        {
            status.Append("  |  waiting for a run to start");
        }
        else
        {
            status.Append(CultureInfo.InvariantCulture,
                $"  |  {s.BoardState ?? "?"}  |  batches {s.Batches}  missing {s.MissingBatches}  invalid {s.InvalidPayloads}");
            if (s.EpochChanges != 0 || s.IndexJumps != 0)
            {
                status.Append(CultureInfo.InvariantCulture, $"  epoch changes {s.EpochChanges}  index jumps {s.IndexJumps}");
            }
            status.Append(CultureInfo.InvariantCulture,
                $"  |  {(s.SamplePeriod > 0 ? 1 / s.SamplePeriod : 0):F1} Hz{(s.TickRateMeasured ? "" : " (nominal tick)")}");
            status.Append(CultureInfo.InvariantCulture,
                $"  |  calibration: accel {(s.AccelCalibrated ? "stored" : "nominal")}, gyro {(s.GyroCalibrated ? "stored" : "nominal")}");
            status.Append(CultureInfo.InvariantCulture,
                $"  |  PC time on {(s.Batches == 0 ? 0 : 100.0 * s.BatchesWithUnixTime / s.Batches):F0} % of batches");
        }
        StreamStatus = status.ToString();

        Quaternion q = Orientation;
        (double roll, double pitch, double yaw) = Euler(q);
        const float toDps = 180f / MathF.PI;
        AttitudeText = string.Create(CultureInfo.InvariantCulture, $"""
            Roll   {roll,8:F1} °
            Pitch  {pitch,8:F1} °
            Yaw    {yaw,8:F1} °

            Quaternion (w x y z)
              {q.W,7:F4} {q.X,7:F4} {q.Y,7:F4} {q.Z,7:F4}

            Accel [m/s²]   |a| {s.Accel.Length() / 9.80665f:F4} g
              {s.Accel.X,8:F3} {s.Accel.Y,8:F3} {s.Accel.Z,8:F3}

            Gyro [dps]
              {s.Gyro.X * toDps,8:F2} {s.Gyro.Y * toDps,8:F2} {s.Gyro.Z * toDps,8:F2}

            Gyro bias [dps]
              {s.GyroBiasDps.X,8:F3} {s.GyroBiasDps.Y,8:F3} {s.GyroBiasDps.Z,8:F3}
              tracked at rest: {s.BiasUpdates} (rejected {s.BiasRejected})
            """);
    }

    private void ShowPositionText(PositionSnapshot p)
    {
        (double roll, double pitch, double yaw) = Euler(p.Orientation);
        PositionText = string.Create(CultureInfo.InvariantCulture, $"""
            Position [m]
              x {p.Position.X,8:F2}
              y {p.Position.Y,8:F2}
              z {p.Position.Z,8:F2}
            Distance   {p.Distance,7:F2} m
            Time       {p.Seconds,7:F1} s

            Velocity [m/s]  {p.VelocityWorld.Length(),5:F2}
              sensor {p.VelocityBody.X,6:F2} {p.VelocityBody.Y,6:F2} {p.VelocityBody.Z,6:F2}
              world  {p.VelocityWorld.X,6:F2} {p.VelocityWorld.Y,6:F2} {p.VelocityWorld.Z,6:F2}

            Roll {roll,6:F1}°  Pitch {pitch,6:F1}°
            Yaw  {yaw,6:F1}°

            Model
              estimates {p.EstimatesPerSecond,5:F1} /s
              skipped   {p.Skipped,5}
              time      {p.InferenceMilliseconds,5:F2} ms
              history   {p.Windows,5} s
            {(p.Windows == 0 ? "waiting for the first second" : "")}{(p.Error is null ? "" : "error: " + p.Error)}
            """);
    }

    private void ShowSpectrumText(SpectrumColumn column)
    {
        int size = column.Db[0].Length * 2;
        double rate = column.BinHz * size;
        Vector3 v = _spectrum.Vertical;
        SpectrumText = string.Create(CultureInfo.InvariantCulture, $"""
            Window     {size / rate * 1000,7:F0} ms
            Resolution {column.BinHz,7:F2} Hz
            Nyquist    {rate / 2,7:F0} Hz

            Vertical (sensor x y z)
              {v.X,6:F3} {v.Y,6:F3} {v.Z,6:F3}
            """);
    }

    private static (double Roll, double Pitch, double Yaw) Euler(Quaternion q)
    {
        const double toDeg = 180.0 / Math.PI;
        double roll = Math.Atan2(2 * (q.W * q.X + q.Y * q.Z), 1 - 2 * (q.X * q.X + q.Y * q.Y));
        double pitch = Math.Asin(Math.Clamp(2 * (q.W * q.Y - q.Z * q.X), -1, 1));
        double yaw = Math.Atan2(2 * (q.W * q.Z + q.X * q.Y), 1 - 2 * (q.Y * q.Y + q.Z * q.Z));
        return (roll * toDeg, pitch * toDeg, yaw * toDeg);
    }

    public void Dispose()
    {
        _timer.Stop();
        _player?.Dispose();
        _pipeline.Sink = null;
        _position?.Dispose();
        if (_writer is not null)
        {
            StopRecording();
        }
        _shell?.Dispose();
        _endpoint?.Dispose();
        _settings.Save();
    }
}
