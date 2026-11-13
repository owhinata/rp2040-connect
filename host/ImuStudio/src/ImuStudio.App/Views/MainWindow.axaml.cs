// SPDX-License-Identifier: Apache-2.0
using System.ComponentModel;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Platform.Storage;
using ImuStudio.App.Controls;
using ImuStudio.App.ViewModels;
using ImuStudio.Core.Processing;

namespace ImuStudio.App.Views;

public partial class MainWindow : Window
{
    private readonly MainViewModel _viewModel;
    private readonly SpectrogramControl[] _spectrograms = new SpectrogramControl[SpectrumAnalyzer.Channels];
    private readonly List<float[]> _channelColumns = new();
    private int _historyIndex = -1;

    public MainWindow() : this(null, null, null, null)
    {
    }

    public MainWindow(string? port, string? board, string? replay, string? view)
    {
        _viewModel = new MainViewModel(port, board);
        InitializeComponent();
        if (string.Equals(view, "spectrum", StringComparison.OrdinalIgnoreCase))
        {
            _viewModel.SelectedView = MainViewModel.SpectrumView;
        }
        if (replay is not null)
        {
            _viewModel.Play(replay);
        }
        DataContext = _viewModel;
        _viewModel.PropertyChanged += OnViewModelPropertyChanged;
        ShowShellPane(_viewModel.IsShellPaneVisible);
        // Acceleration on the left, angular rate on the right; vertical axis on top.
        for (int c = 0; c < _spectrograms.Length; c++)
        {
            SpectrogramControl control = new()
            {
                Title = SpectrumAnalyzer.ChannelNames[c],
                Unit = c < 3 ? "dB re 1 m/s²" : "dB re 1 dps",
                Margin = new Avalonia.Thickness(c < 3 ? 0 : 3, c % 3 == 0 ? 0 : 3, c < 3 ? 3 : 0, 0),
            };
            Grid.SetColumn(control, c / 3);
            Grid.SetRow(control, c % 3);
            SpectrumGrid.Children.Add(control);
            _spectrograms[c] = control;
        }
        ApplySpectrumSettings();
        _viewModel.SpectrumColumns += OnSpectrumColumns;
        Closed += (_, _) =>
        {
            KeepShellPaneHeight();
            _viewModel.Dispose();
        };
    }

    private void ApplySpectrumSettings()
    {
        foreach (SpectrogramControl control in _spectrograms)
        {
            control.SetAxis(_viewModel.SpectrumMaxHz, _viewModel.SpectrumLogFrequency);
            control.SetRange((float)_viewModel.SpectrumMinDb, (float)_viewModel.SpectrumMaxDb);
        }
    }

    private void OnSpectrumColumns(List<SpectrumColumn> columns)
    {
        // The controls take columns of one size at a time; a change of size starts a new picture.
        int bins = columns[^1].Db[0].Length;
        SpectrumColumn last = columns[^1];
        for (int c = 0; c < _spectrograms.Length; c++)
        {
            _channelColumns.Clear();
            foreach (SpectrumColumn column in columns)
            {
                if (column.Db[c].Length == bins)
                {
                    _channelColumns.Add(column.Db[c]);
                }
            }
            _spectrograms[c].Add(_channelColumns, last.BinHz, last.PeakHz[c], last.PeakDb[c]);
        }
    }

    private const double ShellPaneMinHeight = 70;
    private const double ViewsMinHeight = 160;

    /// <summary>Gives the shell pane its row, or takes the row and the bar above it away.</summary>
    private void ShowShellPane(bool visible)
    {
        RowDefinition views = WorkArea.RowDefinitions[0];
        RowDefinition bar = WorkArea.RowDefinitions[1];
        RowDefinition pane = WorkArea.RowDefinitions[2];
        if (!visible)
        {
            KeepShellPaneHeight();
        }
        views.MinHeight = ViewsMinHeight;
        pane.MinHeight = visible ? ShellPaneMinHeight : 0;
        pane.Height = new GridLength(visible ? Math.Max(ShellPaneMinHeight, _viewModel.ShellPaneHeight) : 0);
        bar.Height = new GridLength(visible ? 6 : 0);
        ShellSplitter.IsVisible = visible;
        ShellPane.IsVisible = visible;
    }

    private void KeepShellPaneHeight()
    {
        if (ShellPane.IsVisible && ShellPane.Bounds.Height > 0)
        {
            _viewModel.ShellPaneHeight = WorkArea.RowDefinitions[2].ActualHeight;
        }
    }

    private void OnViewModelPropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        switch (e.PropertyName)
        {
            case nameof(MainViewModel.IsShellPaneVisible):
                ShowShellPane(_viewModel.IsShellPaneVisible);
                break;
            case nameof(MainViewModel.SpectrumMaxFrequency):
            case nameof(MainViewModel.SpectrumLogFrequency):
            case nameof(MainViewModel.SpectrumMinDb):
            case nameof(MainViewModel.SpectrumMaxDb):
                ApplySpectrumSettings();
                break;
            case nameof(MainViewModel.SelectedView):
                // The other view had the samples meanwhile: the old picture is not continuous with what comes.
                foreach (SpectrogramControl control in _spectrograms)
                {
                    control.Clear();
                }
                break;
        }
        if (e.PropertyName == nameof(MainViewModel.ShellText))
        {
            // Keep the newest output in view.
            ShellLogBox.CaretIndex = _viewModel.ShellText.Length;
        }
    }

    private void OnShellInputKeyDown(object? sender, KeyEventArgs e)
    {
        List<string> history = _viewModel.ShellHistory;
        switch (e.Key)
        {
            case Key.Enter:
                _historyIndex = -1;
                _viewModel.SendShellCommand.Execute(null);
                e.Handled = true;
                break;
            case Key.Up when history.Count > 0:
                _historyIndex = _historyIndex < 0 ? history.Count - 1 : Math.Max(0, _historyIndex - 1);
                ShowHistory(history[_historyIndex]);
                e.Handled = true;
                break;
            case Key.Down when _historyIndex >= 0:
                _historyIndex++;
                if (_historyIndex >= history.Count)
                {
                    _historyIndex = -1;
                    ShowHistory("");
                }
                else
                {
                    ShowHistory(history[_historyIndex]);
                }
                e.Handled = true;
                break;
        }
    }

    private void ShowHistory(string line)
    {
        _viewModel.ShellInput = line;
        ShellInputBox.CaretIndex = line.Length;
    }

    private async void OnReplayClick(object? sender, RoutedEventArgs e)
    {
        IStorageFolder? start = await StorageProvider.TryGetFolderFromPathAsync(_viewModel.RecordingsDirectory);
        IReadOnlyList<IStorageFolder> folders = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions
        {
            Title = "Recording to replay (a directory with raw.bin)",
            AllowMultiple = false,
            SuggestedStartLocation = start,
        });
        if (folders.Count == 1 && folders[0].TryGetLocalPath() is string path)
        {
            _viewModel.Play(path);
        }
    }
}
