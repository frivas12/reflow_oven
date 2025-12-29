using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Media;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace ReflowOvenApp;

public class MainForm : Form
{
    private readonly SerialPort serialPort = new();
    private readonly System.Windows.Forms.Timer refreshTimer = new();
    private readonly ComboBox portSelector = new();
    private readonly Button connectButton = new();
    private readonly Button startButton = new();
    private readonly Button abortButton = new();
    private readonly Label statusLabel = new();
    private readonly Label phaseLabel = new();
    private readonly Label phaseCountdownLabel = new();
    private readonly Label setpointStatusLabel = new();
    private readonly Label setpointProfileLabel = new();
    private readonly Label alertLabel = new();
    private readonly Label elapsedLabel = new();
    private readonly Label remainingLabel = new();
    private readonly Label totalLabel = new();
    private readonly Label actualLabel = new();
    private readonly Chart chart = new();
    private readonly DataGridView profileGrid = new();
    private readonly System.Windows.Forms.Timer alertTimer = new();
    private readonly Dictionary<string, int> profileRowLookup = new();

    private DateTime sessionStart = DateTime.MinValue;
    private string currentState = "IDLE";
    private string currentPhase = "IDLE";
    private int alertBeepsRemaining = 0;
    private int totalProfileSeconds = 1;
    private double? profileTimeSeconds = null;

    // Gate plotting/time until temp starts rising (fixes “dead time” / laggy chart start)
    private bool profileArmed = false;
    private double? lastTemp = null;
    private DateTime? lastTempTimestamp = null;

    // How much rise before we “start the clock”
    private const double StartDeltaC = 5.0;

    private sealed record ProfileStep(string Label, int DurationSeconds, double StartTempC, double EndTempC);

    private static readonly ProfileStep[] ProfileSteps =
    {
        new("PREHEAT", 150, 25.0, 150.0),
        new("SOAK", 120, 150.0, 180.0),
        new("REFLOW", 45, 180.0, 225.0),
        new("COOL", 120, 225.0, 50.0)
    };

    public MainForm()
    {
        Text = "Reflow Oven Controller";
        Size = new Size(900, 600);
        MinimumSize = new Size(900, 600);

        this.Icon = new Icon("ReflowOven.ico");

        var mainLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(12)
        };
        mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        portSelector.DropDownStyle = ComboBoxStyle.DropDownList;
        portSelector.Width = 140;

        connectButton.Text = "Connect";
        connectButton.Click += (_, _) => ToggleConnection();

        startButton.Text = "Start";
        startButton.Enabled = false;
        startButton.Click += (_, _) => SendCommand("START");

        abortButton.Text = "Abort";
        abortButton.Enabled = false;
        abortButton.Click += (_, _) => SendCommand("ABORT");

        statusLabel.Text = "Status: Disconnected";
        statusLabel.AutoSize = true;

        phaseLabel.Text = "Phase: -";
        phaseLabel.AutoSize = true;

        phaseCountdownLabel.Text = "Phase Remaining: --:--";
        phaseCountdownLabel.AutoSize = true;

        setpointStatusLabel.Text = "Setpoint: -- °C";
        setpointStatusLabel.AutoSize = true;

        actualLabel.Text = "Actual: -- °C";
        actualLabel.AutoSize = true;

        setpointProfileLabel.Text = "Setpoint: -- °C";
        setpointProfileLabel.AutoSize = true;

        alertLabel.Text = "REFLOW COMPLETE — OPEN OVEN DOOR";
        alertLabel.AutoSize = true;
        alertLabel.ForeColor = Color.Firebrick;
        alertLabel.Visible = false;

        elapsedLabel.Text = "Elapsed: --:--";
        elapsedLabel.AutoSize = true;

        remainingLabel.Text = "Remaining: --:--";
        remainingLabel.AutoSize = true;

        totalLabel.Text = "Total: --:--";
        totalLabel.AutoSize = true;

        var controlsPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = false,
            FlowDirection = FlowDirection.LeftToRight,
            Padding = new Padding(0),
            Margin = new Padding(0)
        };
        controlsPanel.Controls.Add(portSelector);
        controlsPanel.Controls.Add(connectButton);
        controlsPanel.Controls.Add(startButton);
        controlsPanel.Controls.Add(abortButton);

        var statusPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = false,
            FlowDirection = FlowDirection.TopDown,
            Padding = new Padding(0),
            Margin = new Padding(0)
        };
        statusPanel.Controls.Add(statusLabel);
        statusPanel.Controls.Add(phaseLabel);
        statusPanel.Controls.Add(phaseCountdownLabel);
        statusPanel.Controls.Add(setpointStatusLabel);
        statusPanel.Controls.Add(actualLabel);
        statusPanel.Controls.Add(alertLabel);

        var headerLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1
        };
        headerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70));
        headerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
        headerLayout.Controls.Add(controlsPanel, 0, 0);
        headerLayout.Controls.Add(statusPanel, 1, 0);

        var contentLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            RowCount = 1
        };
        contentLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 260));
        contentLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));

        var profileGroup = new GroupBox
        {
            Text = "Profile",
            Dock = DockStyle.Fill
        };

        profileGrid.AllowUserToAddRows = false;
        profileGrid.AllowUserToDeleteRows = false;
        profileGrid.AllowUserToResizeRows = false;
        profileGrid.ReadOnly = true;
        profileGrid.RowHeadersVisible = false;
        profileGrid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
        profileGrid.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
        profileGrid.MultiSelect = false;
        profileGrid.Dock = DockStyle.Fill;

        var profileLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 5,
            Padding = new Padding(6)
        };
        profileLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        profileLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        profileLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        profileLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        profileLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        profileLayout.Controls.Add(profileGrid, 0, 0);
        profileLayout.Controls.Add(totalLabel, 0, 1);
        profileLayout.Controls.Add(elapsedLabel, 0, 2);
        profileLayout.Controls.Add(remainingLabel, 0, 3);
        profileLayout.Controls.Add(setpointProfileLabel, 0, 4);
        profileGroup.Controls.Add(profileLayout);

        chart.Dock = DockStyle.Fill;
        chart.ChartAreas.Add(CreateChartArea());
        chart.Series.Add(CreateProfileSeries());
        chart.Series.Add(CreateSeries("Setpoint", Color.OrangeRed));
        chart.Series.Add(CreateSeries("Actual", Color.SteelBlue));
        chart.Legends.Add(new Legend());

        contentLayout.Controls.Add(profileGroup, 0, 0);
        contentLayout.Controls.Add(chart, 1, 0);

        mainLayout.Controls.Add(headerLayout, 0, 0);
        mainLayout.Controls.Add(contentLayout, 0, 1);

        Controls.Add(mainLayout);

        refreshTimer.Interval = 200;
        refreshTimer.Tick += (_, _) =>
        {
            PumpSerial();
            UpdateCycleTime();
        };
        refreshTimer.Start();

        alertTimer.Interval = 400;
        alertTimer.Tick += (_, _) =>
        {
            if (alertBeepsRemaining > 0)
            {
                SystemSounds.Beep.Play();
                alertBeepsRemaining -= 1;
            }

            if (alertBeepsRemaining <= 0)
            {
                alertTimer.Stop();
            }
        };

        serialPort.BaudRate = 115200;
        serialPort.NewLine = "\n";

        InitializeProfileView();
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
        area.AxisX.LabelStyle.Format = "0.0";
        area.AxisY.LabelStyle.Format = "0.0";
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

    private static Series CreateProfileSeries()
    {
        return new Series("Profile")
        {
            ChartType = SeriesChartType.Line,
            BorderWidth = 2,
            Color = Color.DimGray,
            BorderDashStyle = ChartDashStyle.Dash
        };
    }

    private void InitializeProfileView()
    {
        profileGrid.Columns.Clear();
        profileGrid.Columns.Add("Phase", "Phase");
        profileGrid.Columns.Add("Duration", "Duration");
        profileGrid.Columns.Add("Start", "Start (°C)");
        profileGrid.Columns.Add("End", "End (°C)");

        profileGrid.Rows.Clear();
        profileRowLookup.Clear();

        int rowIndex = 0;
        foreach (var step in ProfileSteps)
        {
            profileGrid.Rows.Add(
                step.Label,
                FormatSeconds(step.DurationSeconds),
                step.StartTempC.ToString("0.0", CultureInfo.InvariantCulture),
                step.EndTempC.ToString("0.0", CultureInfo.InvariantCulture));
            profileRowLookup[step.Label] = rowIndex;
            rowIndex += 1;
        }

        totalProfileSeconds = 0;
        foreach (var step in ProfileSteps)
        {
            totalProfileSeconds += step.DurationSeconds;
        }

        totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
        RenderProfileSeries();
    }

    private void RenderProfileSeries()
    {
        var series = chart.Series["Profile"];
        series.Points.Clear();
        double elapsed = 0;
        if (ProfileSteps.Length == 0)
        {
            return;
        }

        series.Points.AddXY(elapsed, ProfileSteps[0].StartTempC);
        foreach (var step in ProfileSteps)
        {
            elapsed += step.DurationSeconds;
            series.Points.AddXY(elapsed, step.EndTempC);
        }
    }

    private static string FormatSeconds(int seconds)
    {
        var span = TimeSpan.FromSeconds(seconds);
        return span.ToString(@"mm\:ss", CultureInfo.InvariantCulture);
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
        currentState = "IDLE";
        currentPhase = "IDLE";
        alertLabel.Visible = false;
        alertBeepsRemaining = 0;
        alertTimer.Stop();
        profileArmed = false;
        lastTemp = null;
        lastTempTimestamp = null;
        profileTimeSeconds = null;
        ResetSetpointLabels();
        ResetActualLabel();
        UpdateCycleTime();
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
                sessionStart = DateTime.MinValue;
                profileArmed = false;
                lastTemp = null;
                lastTempTimestamp = null;
                profileTimeSeconds = null;
                chart.Series["Setpoint"].Points.Clear();
                chart.Series["Actual"].Points.Clear();
                alertLabel.Visible = false;
                alertBeepsRemaining = 0;
                alertTimer.Stop();
                ResetSetpointLabels();
                ResetActualLabel();
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
        double? profileTime = null;
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
            else if (part.StartsWith("PT:", StringComparison.OrdinalIgnoreCase) &&
                     double.TryParse(part[3..], NumberStyles.Float, CultureInfo.InvariantCulture, out var pt))
            {
                profileTime = pt;
            }
        }

        if (!string.IsNullOrEmpty(state))
        {
            currentState = state;
            statusLabel.Text = $"Status: {state}";

            if (currentState.Equals("RUNNING", StringComparison.OrdinalIgnoreCase))
            {
                if (!profileArmed)
                {
                    profileArmed = true;
                    sessionStart = DateTime.MinValue;
                    lastTemp = null;
                    lastTempTimestamp = null;
                }
            }

            if (currentState.Equals("IDLE", StringComparison.OrdinalIgnoreCase))
            {
                sessionStart = DateTime.MinValue;
                profileArmed = false;
                lastTemp = null;
                lastTempTimestamp = null;
                profileTimeSeconds = null;
            }
            else if (currentState.Equals("DONE", StringComparison.OrdinalIgnoreCase))
            {
                profileArmed = false;
                lastTemp = null;
                lastTempTimestamp = null;
            }
        }

        if (!string.IsNullOrEmpty(phase))
        {
            var previousPhase = currentPhase;
            currentPhase = phase;
            phaseLabel.Text = $"Phase: {phase}";
            HighlightCurrentPhase(phase);
            HandlePhaseChange(previousPhase, phase);
        }

        if (profileTime.HasValue)
        {
            profileTimeSeconds = profileTime.Value;
        }

        bool isCooling = currentPhase.Equals("COOL", StringComparison.OrdinalIgnoreCase);

        if (temp.HasValue)
        {
            actualLabel.Text = $"Actual: {temp.Value:0.0} °C";
        }

        // Start timing only after temp rises by StartDeltaC
        if (profileArmed && sessionStart == DateTime.MinValue && temp.HasValue)
        {
            if (lastTemp.HasValue)
            {
                if (temp.Value >= lastTemp.Value + StartDeltaC)
                {
                    sessionStart = DateTime.UtcNow;
                }
            }
            lastTemp = temp.Value;
            lastTempTimestamp = DateTime.UtcNow;
        }

        if (temp.HasValue && setpoint.HasValue)
        {
            var plotTime = GetPlotTimeSeconds();
            if (plotTime.HasValue)
            {
                chart.Series["Setpoint"].Points.AddXY(plotTime.Value, setpoint.Value);
                chart.Series["Actual"].Points.AddXY(plotTime.Value, temp.Value);
            }

            if (isCooling)
            {
                ResetSetpointLabels();
            }
            else
            {
                UpdateSetpointLabels(setpoint.Value);
            }
        }

        UpdatePhaseCountdown();
    }

    private void HandlePhaseChange(string previousPhase, string newPhase)
    {
        var isCooling = newPhase.Equals("COOL", StringComparison.OrdinalIgnoreCase);
        alertLabel.Visible = isCooling;
        if (isCooling)
        {
            setpointStatusLabel.ForeColor = SystemColors.GrayText;
            setpointProfileLabel.ForeColor = SystemColors.GrayText;
            if (!previousPhase.Equals("COOL", StringComparison.OrdinalIgnoreCase))
            {
                alertBeepsRemaining = 3;
                alertTimer.Start();
            }
        }
        else
        {
            setpointStatusLabel.ForeColor = SystemColors.ControlText;
            setpointProfileLabel.ForeColor = SystemColors.ControlText;
        }
    }

    private void HighlightCurrentPhase(string phase)
    {
        if (!profileRowLookup.TryGetValue(phase, out var rowIndex))
        {
            profileGrid.ClearSelection();
            return;
        }

        profileGrid.ClearSelection();
        profileGrid.Rows[rowIndex].Selected = true;
    }

    private void UpdateSetpointLabels(double setpoint)
    {
        var text = $"Setpoint: {setpoint:0.0} °C";
        setpointStatusLabel.Text = text;
        setpointProfileLabel.Text = text;
    }

    private void ResetSetpointLabels()
    {
        setpointStatusLabel.Text = "Setpoint: -- °C";
        setpointProfileLabel.Text = "Setpoint: -- °C";
        setpointStatusLabel.ForeColor = SystemColors.ControlText;
        setpointProfileLabel.ForeColor = SystemColors.ControlText;
    }

    private void ResetActualLabel()
    {
        actualLabel.Text = "Actual: -- °C";
    }

    private void UpdateCycleTime()
    {
        if (!currentState.Equals("RUNNING", StringComparison.OrdinalIgnoreCase) &&
            !currentState.Equals("DONE", StringComparison.OrdinalIgnoreCase))
        {
            elapsedLabel.Text = "Elapsed: --:--";
            remainingLabel.Text = "Remaining: --:--";
            if (totalProfileSeconds > 0)
            {
                totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
            }
            phaseCountdownLabel.Text = "Phase Remaining: --:--";
            return;
        }

        var elapsedSecondsRaw = GetElapsedProfileSeconds();
        if (!elapsedSecondsRaw.HasValue)
        {
            return;
        }

        var elapsedSeconds = (int)Math.Max(0, elapsedSecondsRaw.Value);
        if (elapsedSeconds > totalProfileSeconds)
        {
            elapsedSeconds = totalProfileSeconds;
        }

        elapsedLabel.Text = $"Elapsed: {FormatSeconds(elapsedSeconds)}";
        var remainingSeconds = Math.Max(0, totalProfileSeconds - elapsedSeconds);
        remainingLabel.Text = $"Remaining: {FormatSeconds(remainingSeconds)}";
        totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
    }

    private double? GetPlotTimeSeconds()
    {
        if (profileTimeSeconds.HasValue)
        {
            return Math.Max(0, profileTimeSeconds.Value);
        }

        if (sessionStart == DateTime.MinValue)
        {
            return null;
        }

        return Math.Max(0, (DateTime.UtcNow - sessionStart).TotalSeconds);
    }

    private double? GetElapsedProfileSeconds()
    {
        if (profileTimeSeconds.HasValue)
        {
            return Math.Max(0, Math.Min(profileTimeSeconds.Value, totalProfileSeconds));
        }

        if (sessionStart == DateTime.MinValue)
        {
            return null;
        }

        return Math.Max(0, (DateTime.UtcNow - sessionStart).TotalSeconds);
    }

    private void UpdatePhaseCountdown()
    {
        if (!profileTimeSeconds.HasValue ||
            !currentState.Equals("RUNNING", StringComparison.OrdinalIgnoreCase))
        {
            phaseCountdownLabel.Text = "Phase Remaining: --:--";
            return;
        }

        if (!TryGetProfileStep(currentPhase, out var step, out var index))
        {
            phaseCountdownLabel.Text = "Phase Remaining: --:--";
            return;
        }

        double phaseStart = 0;
        for (int i = 0; i < index; i++)
        {
            phaseStart += ProfileSteps[i].DurationSeconds;
        }

        var phaseEnd = phaseStart + step.DurationSeconds;
        var remainingSeconds = Math.Max(0, phaseEnd - profileTimeSeconds.Value);
        var remaining = (int)Math.Max(0, Math.Ceiling(remainingSeconds));
        phaseCountdownLabel.Text = $"Phase Remaining: {FormatSeconds(remaining)}";
    }

    private static bool TryGetProfileStep(string phase, out ProfileStep step, out int index)
    {
        for (int i = 0; i < ProfileSteps.Length; i++)
        {
            if (ProfileSteps[i].Label.Equals(phase, StringComparison.OrdinalIgnoreCase))
            {
                step = ProfileSteps[i];
                index = i;
                return true;
            }
        }

        step = new ProfileStep(string.Empty, 0, 0, 0);
        index = -1;
        return false;
    }
}
