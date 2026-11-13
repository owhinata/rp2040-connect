// SPDX-License-Identifier: Apache-2.0
using System.ComponentModel;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Platform.Storage;
using ImuStudio.App.ViewModels;

namespace ImuStudio.App.Views;

public partial class MainWindow : Window
{
    private readonly MainViewModel _viewModel;
    private int _historyIndex = -1;

    public MainWindow() : this(null, null, null)
    {
    }

    public MainWindow(string? port, string? board, string? replay)
    {
        _viewModel = new MainViewModel(port, board);
        InitializeComponent();
        if (replay is not null)
        {
            _viewModel.Play(replay);
        }
        DataContext = _viewModel;
        _viewModel.PropertyChanged += OnViewModelPropertyChanged;
        ShowShellPane(_viewModel.IsShellPaneVisible);
        Closed += (_, _) =>
        {
            KeepShellPaneHeight();
            _viewModel.Dispose();
        };
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
        if (e.PropertyName == nameof(MainViewModel.IsShellPaneVisible))
        {
            ShowShellPane(_viewModel.IsShellPaneVisible);
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
