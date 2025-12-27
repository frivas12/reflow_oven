using System;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Windows.Forms.DataVisualization.Charting;


namespace ReflowOvenApp;

public class MainForm : Form
{
    private readonly SerialPort serialPort = new();
    private readonly System.Windows.Forms.Timer refreshTimer = new();
    private readonly Chart chart = new();
    private readonly ComboBox portSelector = new();
    private readonly Button connectButton = new();
    private readonly Button startButton = new();
    private readonly Button abortButton = new();
    private readonly Label statusLabel = new();
    private readonly Label phaseLabel = new();

    private DateTime sessionStart = DateTime.MinValue;

    public MainForm()
    {
        Text = "Reflow Oven Controller";
        Size = new Size(900, 600);
        MinimumSize = new Size(900, 600);

        portSelector.DropDownStyle = ComboBoxStyle.DropDownList;
        portSelector.Location = new Point(20, 20);
        portSelector.Width = 120;

        connectButton.Text = "Connect";
        connectButton.Location = new Point(160, 18);
        connectButton.Click += (_, _) => ToggleConnection();

        startButton.Text = "Start Reflow";
        startButton.Location = new Point(280, 18);
        startButton.Enabled = false;
        startButton.Click += (_, _) => SendCommand("START");

        abortButton.Text = "Abort";
        abortButton.Location = new Point(400, 18);
        abortButton.Enabled = false;
        abortButton.Click += (_, _) => SendCommand("ABORT");

        statusLabel.Text = "Status: Disconnected";
        statusLabel.Location = new Point(520, 22);
        statusLabel.AutoSize = true;

        phaseLabel.Text = "Phase: -";
        phaseLabel.Location = new Point(520, 44);
        phaseLabel.AutoSize = true;

        chart.Location = new Point(20, 70);
        chart.Size = new Size(840, 460);
        chart.ChartAreas.Add(CreateChartArea());
        chart.Series.Add(CreateSeries("Setpoint", Color.OrangeRed));
        chart.Series.Add(CreateSeries("Actual", Color.SteelBlue));
        chart.Legends.Add(new Legend());

        Controls.Add(portSelector);
        Controls.Add(connectButton);
        Controls.Add(startButton);
        Controls.Add(abortButton);
        Controls.Add(statusLabel);
        Controls.Add(phaseLabel);
        Controls.Add(chart);

        refreshTimer.Interval = 200;
        refreshTimer.Tick += (_, _) => PumpSerial();
        refreshTimer.Start();

        serialPort.BaudRate = 115200;
        serialPort.NewLine = "\n";

        Load += (_, _) => RefreshPorts();
        FormClosing += (_, _) => CloseSerial();
    }

    private static ChartArea CreateChartArea()
    {
        var area = new ChartArea("Reflow");
        area.AxisX.Title = "Time (s)";
        area.AxisY.Title = "Temperature (°C)";
        area.AxisX.MajorGrid.LineColor = Color.Gainsboro;
        area.AxisY.MajorGrid.LineColor = Color.Gainsboro;
        return area;
    }

    private static Series CreateSeries(string name, Color color)
    {
        return new Series(name)
        {
            ChartType = SeriesChartType.Line,
            BorderWidth = 2,
            Color = color
        };
    }

    private void RefreshPorts()
    {
        portSelector.Items.Clear();
        foreach (var port in SerialPort.GetPortNames())
        {
            portSelector.Items.Add(port);
        }

        if (portSelector.Items.Count > 0)
        {
            portSelector.SelectedIndex = 0;
        }
    }

    private void ToggleConnection()
    {
        if (serialPort.IsOpen)
        {
            CloseSerial();
            return;
        }

        if (portSelector.SelectedItem is not string portName)
        {
            MessageBox.Show("Select a COM port first.");
            return;
        }

        try
        {
            serialPort.PortName = portName;
            serialPort.Open();
            connectButton.Text = "Disconnect";
            statusLabel.Text = $"Status: Connected to {portName}";
            startButton.Enabled = true;
            abortButton.Enabled = true;
            sessionStart = DateTime.UtcNow;
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to open {portName}: {ex.Message}");
        }
    }

    private void CloseSerial()
    {
        if (serialPort.IsOpen)
        {
            serialPort.Close();
        }

        connectButton.Text = "Connect";
        statusLabel.Text = "Status: Disconnected";
        startButton.Enabled = false;
        abortButton.Enabled = false;
    }

    private void SendCommand(string command)
    {
        if (!serialPort.IsOpen)
        {
            MessageBox.Show("Connect to the Arduino first.");
            return;
        }

        try
        {
            serialPort.WriteLine(command);
            if (command == "START")
            {
                sessionStart = DateTime.UtcNow;
                chart.Series["Setpoint"].Points.Clear();
                chart.Series["Actual"].Points.Clear();
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Serial write failed: {ex.Message}");
        }
    }

    private void PumpSerial()
    {
        if (!serialPort.IsOpen)
        {
            return;
        }

        try
        {
            while (serialPort.BytesToRead > 0)
            {
                var line = serialPort.ReadLine().Trim();
                if (line.Length == 0)
                {
                    continue;
                }
                ParseTelemetry(line);
            }
        }
        catch (Exception ex)
        {
            statusLabel.Text = $"Status: Serial error ({ex.Message})";
        }
    }

    private void ParseTelemetry(string line)
    {
        // Expected format: T:123.45;S:150.00;STATE:RUNNING;PHASE:PREHEAT
        var parts = line.Split(';', StringSplitOptions.RemoveEmptyEntries);
        double? temp = null;
        double? setpoint = null;
        string? state = null;
        string? phase = null;

        foreach (var part in parts)
        {
            if (part.StartsWith("T:", StringComparison.OrdinalIgnoreCase) &&
                double.TryParse(part[2..], NumberStyles.Float, CultureInfo.InvariantCulture, out var t))
            {
                temp = t;
            }
            else if (part.StartsWith("S:", StringComparison.OrdinalIgnoreCase) &&
                     double.TryParse(part[2..], NumberStyles.Float, CultureInfo.InvariantCulture, out var s))
            {
                setpoint = s;
            }
            else if (part.StartsWith("STATE:", StringComparison.OrdinalIgnoreCase))
            {
                state = part[6..];
            }
            else if (part.StartsWith("PHASE:", StringComparison.OrdinalIgnoreCase))
            {
                phase = part[6..];
            }
        }

        if (temp.HasValue && setpoint.HasValue)
        {
            var elapsedSeconds = (DateTime.UtcNow - sessionStart).TotalSeconds;
            chart.Series["Setpoint"].Points.AddXY(elapsedSeconds, setpoint.Value);
            chart.Series["Actual"].Points.AddXY(elapsedSeconds, temp.Value);
        }

        if (!string.IsNullOrEmpty(state))
        {
            statusLabel.Text = $"Status: {state}";
        }

        if (!string.IsNullOrEmpty(phase))
        {
            phaseLabel.Text = $"Phase: {phase}";
        }
    }
}
