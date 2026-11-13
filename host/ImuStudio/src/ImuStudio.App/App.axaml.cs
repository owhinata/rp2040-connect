using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;

namespace ImuStudio.App;

public partial class App : Application
{
    public override void Initialize() => AvaloniaXamlLoader.Load(this);

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            // --port N: MQTT port; --board ADDRESS: connect the shell; --replay DIR: replay a recording;
            // --view attitude|spectrum: the view to start with.
            string[] args = desktop.Args ?? [];
            desktop.MainWindow = new Views.MainWindow(Option(args, "--port"), Option(args, "--board"),
                                                      Option(args, "--replay"), Option(args, "--view"));
        }

        base.OnFrameworkInitializationCompleted();
    }

    private static string? Option(string[] args, string name)
    {
        int i = Array.IndexOf(args, name);
        return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
    }
}
