using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Media;
using System.Text.Json;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;

namespace ReflowOvenApp;

public class MainForm : Form
{
    private readonly SerialPort serialPort = new();
    private System.Windows.Forms.Timer refreshTimer = new();
    private System.Windows.Forms.Timer alertTimer = new();

    private readonly ComboBox portSelector = new();
    private readonly Button connectButton = new();
    private readonly Button startButton = new();
    private readonly Button abortButton = new();
    private readonly Button heaterToggleButton = new();
    private readonly Button saveChartButton = new();
    private readonly Button saveProfileButton = new();
    private readonly Button loadProfileButton = new();
    private readonly Button resetProfileButton = new();

    private readonly Label statusLabel = new();
    private readonly Label phaseLabel = new();
    private readonly Label phaseCountdownLabel = new();
    private readonly Label setpointStatusLabel = new();
    private readonly Label actualStatusLabel = new();
    private readonly Label alertLabel = new();

    private readonly Label totalLabel = new();
    private readonly Label elapsedLabel = new();
    private readonly Label remainingLabel = new();

    private readonly Chart chart = new();
    private readonly DataGridView profileGrid = new();

    private int alertBeepsRemaining = 0;

    // ---------- Beep-stop logic ----------
    private double? lastActualTemp = null;
    private DateTime lastActualTimeUtc = DateTime.MinValue;
    private double filteredSlopeCps = 0.0; // degC / sec
    private double peakTempC = double.MinValue;

    private bool beepArmed = false;
    private bool heaterManualOn = false;

    // Slope detection tuning
    private const double SlopeFilterAlpha = 0.25;
    private const double CoolingSlopeThreshold = -0.10; // C/s
    private const double XAxisBufferSeconds = 10;

    // "Reflow reached" is based on peak temperature achieved
    private const double ReflowReachedPeakTempC = 205.0; // adjust if you want stricter

    // Profile definition (UI only; firmware is source of truth)
    private sealed record ProfileStep(string Label, int DurationSeconds, double StartTempC, double EndTempC, double OffsetC);

    private static readonly ProfileStep[] DefaultProfileSteps =
    {
        new("PREHEAT", 150, 25.0, 150.0, 0.0),
        new("SOAK",   120, 150.0, 180.0, 0.0),
        new("REFLOW",  45, 180.0, 225.0, 0.0),
        new("COOL",   120, 225.0, 50.0, 0.0)
    };

    private readonly List<ProfileStep> profileSteps = new();
    private readonly Dictionary<string, int> profileRowLookup = new();
    private int totalProfileSeconds = 1;

    private string currentState = "IDLE";
    private string currentPhase = "IDLE";

    // We prefer PT from firmware for X axis + elapsed/remaining
    private double? profileTimeSeconds = null;

    // Fallback wall time if PT missing
    private DateTime sessionStart = DateTime.MinValue;
    private readonly string settingsFilePath;
    private string? lastPortName;

    public MainForm()
    {
        Text = "Reflow Oven Controller";
        Icon = new Icon(Path.Combine(AppContext.BaseDirectory, "Assets", "reflowOvenIcon.ico"));
        Size = new Size(900, 600);
        MinimumSize = new Size(900, 600);

        settingsFilePath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "ReflowOvenApp",
            "settings.txt");
        LoadSettings();

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

        heaterToggleButton.Text = "Heater: Off";
        heaterToggleButton.Enabled = false;
        heaterToggleButton.Click += (_, _) => ToggleHeater();

        saveChartButton.Text = "Save PNG";
        saveChartButton.Click += (_, _) => SaveChartImage();

        saveProfileButton.Text = "Save Profile";
        saveProfileButton.Click += (_, _) => SaveProfile();

        loadProfileButton.Text = "Load Profile";
        loadProfileButton.Click += (_, _) => LoadProfile();

        resetProfileButton.Text = "Reset Profile";
        resetProfileButton.Click += (_, _) => ResetProfileToDefault();

        var controlsPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            WrapContents = false,
            FlowDirection = FlowDirection.LeftToRight
        };
        controlsPanel.Controls.Add(portSelector);
        controlsPanel.Controls.Add(connectButton);
        controlsPanel.Controls.Add(startButton);
        controlsPanel.Controls.Add(abortButton);
        controlsPanel.Controls.Add(heaterToggleButton);
        controlsPanel.Controls.Add(saveChartButton);

        statusLabel.Text = "Status: Disconnected";
        statusLabel.AutoSize = true;

        phaseLabel.Text = "Phase: -";
        phaseLabel.AutoSize = true;

        phaseCountdownLabel.Text = "Phase Remaining: --:--";
        phaseCountdownLabel.AutoSize = true;

        setpointStatusLabel.Text = "Setpoint: -- °C";
        setpointStatusLabel.AutoSize = true;

        actualStatusLabel.Text = "Actual: -- °C";
        actualStatusLabel.AutoSize = true;

        alertLabel.Text = "REFLOW COMPLETE — OPEN OVEN DOOR";
        alertLabel.AutoSize = true;
        alertLabel.ForeColor = Color.Firebrick;
        alertLabel.Visible = false;

        var statusPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            WrapContents = false,
            FlowDirection = FlowDirection.TopDown
        };
        statusPanel.Controls.Add(statusLabel);
        statusPanel.Controls.Add(phaseLabel);
        statusPanel.Controls.Add(phaseCountdownLabel);
        statusPanel.Controls.Add(setpointStatusLabel);
        statusPanel.Controls.Add(actualStatusLabel);
        statusPanel.Controls.Add(alertLabel);

        profileGrid.AllowUserToAddRows = false;
        profileGrid.AllowUserToDeleteRows = false;
        profileGrid.AllowUserToResizeRows = false;
        profileGrid.ReadOnly = false;
        profileGrid.RowHeadersVisible = false;
        profileGrid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
        profileGrid.SelectionMode = DataGridViewSelectionMode.FullRowSelect;
        profileGrid.MultiSelect = false;
        profileGrid.Dock = DockStyle.Fill;
        profileGrid.CellEndEdit += ProfileGridCellEndEdit;

        var profileGroup = new GroupBox { Text = "Profile", Dock = DockStyle.Fill };

        totalLabel.Text = "Total: --:--";
        totalLabel.AutoSize = true;

        elapsedLabel.Text = "Elapsed: --:--";
        elapsedLabel.AutoSize = true;

        remainingLabel.Text = "Remaining: --:--";
        remainingLabel.AutoSize = true;

        var profileButtonsPanel = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            WrapContents = false,
            FlowDirection = FlowDirection.LeftToRight
        };
        profileButtonsPanel.Controls.Add(saveProfileButton);
        profileButtonsPanel.Controls.Add(loadProfileButton);
        profileButtonsPanel.Controls.Add(resetProfileButton);

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
        profileLayout.Controls.Add(profileButtonsPanel, 0, 1);
        profileLayout.Controls.Add(totalLabel, 0, 2);
        profileLayout.Controls.Add(elapsedLabel, 0, 3);
        profileLayout.Controls.Add(remainingLabel, 0, 4);
        profileGroup.Controls.Add(profileLayout);

        chart.Dock = DockStyle.Fill;
        chart.ChartAreas.Add(CreateChartArea());
        chart.Legends.Add(new Legend());
        chart.Series.Add(CreateProfileSeries());
        chart.Series.Add(CreateSeries("Setpoint", Color.OrangeRed));
        chart.Series.Add(CreateSeries("Actual", Color.SteelBlue));

        var mainLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 2,
            Padding = new Padding(12)
        };
        mainLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        mainLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));

        var headerLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
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
                alertBeepsRemaining--;
            }

            if (alertBeepsRemaining <= 0)
                alertTimer.Stop();
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
        area.AxisX.Title = "Time (min)";
        area.AxisY.Title = "Temperature (°C)";
        area.AxisX.MajorGrid.LineColor = Color.Gainsboro;
        area.AxisY.MajorGrid.LineColor = Color.Gainsboro;
        area.AxisX.LabelStyle.Format = "0.0";
        area.AxisY.LabelStyle.Format = "0.0";

        area.CursorX.IsUserEnabled = false;
        area.CursorX.IsUserSelectionEnabled = false;
        area.CursorY.IsUserEnabled = false;
        area.CursorY.IsUserSelectionEnabled = false;
        area.AxisX.ScaleView.Zoomable = false;
        area.AxisY.ScaleView.Zoomable = false;

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
        profileGrid.Columns.Add("Offset", "Offset (°C)");

        profileGrid.Rows.Clear();
        profileRowLookup.Clear();
        profileSteps.Clear();
        profileSteps.AddRange(DefaultProfileSteps);
        LoadProfileRows();

        for (int i = 0; i < profileGrid.Columns.Count; i++)
            profileGrid.Columns[i].ReadOnly = i == 0;
    }

    private void ResetXAxisToFullProfile()
    {
        var area = chart.ChartAreas["Reflow"];
        area.AxisX.Minimum = 0;
        area.AxisX.Maximum = Math.Max(1, ToMinutes(totalProfileSeconds));
        try
        {
            area.AxisX.ScaleView.ZoomReset(0);
            area.AxisY.ScaleView.ZoomReset(0);
        }
        catch { }
    }

    private void RenderProfileSeries()
    {
        var series = chart.Series["Profile"];
        series.Points.Clear();

        double elapsed = 0;
        if (profileSteps.Count == 0) return;

        series.Points.AddXY(ToMinutes(elapsed), GetOffsetTemperature(profileSteps[0], profileSteps[0].StartTempC));
        foreach (var step in profileSteps)
        {
            elapsed += step.DurationSeconds;
            series.Points.AddXY(ToMinutes(elapsed), GetOffsetTemperature(step, step.EndTempC));
        }
    }

    private double GetOffsetTemperature(ProfileStep step, double temperature)
    {
        return temperature + step.OffsetC;
    }

    private void ProfileGridCellEndEdit(object? sender, DataGridViewCellEventArgs e)
    {
        if (e.RowIndex < 0 || e.ColumnIndex == 0) return;
        if (e.RowIndex >= profileSteps.Count) return;

        var row = profileGrid.Rows[e.RowIndex];
        var step = profileSteps[e.RowIndex];

        switch (e.ColumnIndex)
        {
            case 1:
                if (!TryParseDuration(row.Cells[1].Value?.ToString(), out var durationSeconds))
                {
                    MessageBox.Show("Duration must be in mm:ss or a number of seconds.");
                    row.Cells[1].Value = FormatSeconds(step.DurationSeconds);
                    return;
                }
                step = step with { DurationSeconds = durationSeconds };
                row.Cells[1].Value = FormatSeconds(durationSeconds);
                break;
            case 2:
                if (!TryParseTemperature(row.Cells[2].Value?.ToString(), out var startTemp))
                {
                    MessageBox.Show("Start temperature must be a number (e.g. 150.0).");
                    row.Cells[2].Value = step.StartTempC.ToString("0.0", CultureInfo.InvariantCulture);
                    return;
                }
                step = step with { StartTempC = startTemp };
                row.Cells[2].Value = startTemp.ToString("0.0", CultureInfo.InvariantCulture);
                break;
            case 3:
                if (!TryParseTemperature(row.Cells[3].Value?.ToString(), out var endTemp))
                {
                    MessageBox.Show("End temperature must be a number (e.g. 180.0).");
                    row.Cells[3].Value = step.EndTempC.ToString("0.0", CultureInfo.InvariantCulture);
                    return;
                }
                step = step with { EndTempC = endTemp };
                row.Cells[3].Value = endTemp.ToString("0.0", CultureInfo.InvariantCulture);
                break;
            case 4:
                if (!TryParseTemperature(row.Cells[4].Value?.ToString(), out var offset))
                {
                    MessageBox.Show("Offset must be a number (e.g. 5.0).");
                    row.Cells[4].Value = step.OffsetC.ToString("0.0", CultureInfo.InvariantCulture);
                    return;
                }
                step = step with { OffsetC = offset };
                row.Cells[4].Value = offset.ToString("0.0", CultureInfo.InvariantCulture);
                break;
            default:
                return;
        }

        profileSteps[e.RowIndex] = step;
        UpdateProfileTotalsAndRender();
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
            portSelector.Items.Add(port);

        if (portSelector.Items.Count > 0)
        {
            if (!string.IsNullOrWhiteSpace(lastPortName) && portSelector.Items.Contains(lastPortName))
                portSelector.SelectedItem = lastPortName;
            else
                portSelector.SelectedIndex = 0;
        }
    }

    private void ToggleConnection()
    {
        if (serialPort.IsOpen) { CloseSerial(); return; }

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
            heaterToggleButton.Enabled = true;

            sessionStart = DateTime.UtcNow;
            SaveLastPort(portName);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to open {portName}: {ex.Message}");
        }
    }

    private void CloseSerial()
    {
        if (serialPort.IsOpen) serialPort.Close();

        connectButton.Text = "Connect";
        statusLabel.Text = "Status: Disconnected";
        startButton.Enabled = false;
        abortButton.Enabled = false;
        heaterToggleButton.Enabled = false;

        currentState = "IDLE";
        currentPhase = "IDLE";
        profileTimeSeconds = null;
        sessionStart = DateTime.MinValue;

        alertLabel.Visible = false;
        alertBeepsRemaining = 0;
        alertTimer.Stop();

        SetHeaterManualState(false);
        setpointStatusLabel.Text = "Setpoint: -- °C";
        actualStatusLabel.Text = "Actual: -- °C";
        phaseLabel.Text = "Phase: -";
        phaseCountdownLabel.Text = "Phase Remaining: --:--";

        ResetBeepStopState();
        UpdateCycleTime();
    }

    private void ResetBeepStopState()
    {
        lastActualTemp = null;
        lastActualTimeUtc = DateTime.MinValue;
        filteredSlopeCps = 0.0;
        peakTempC = double.MinValue;
        beepArmed = false;
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

            if (command.Equals("START", StringComparison.OrdinalIgnoreCase))
            {
                SetHeaterManualState(false);
                chart.Series["Setpoint"].Points.Clear();
                chart.Series["Actual"].Points.Clear();

                profileTimeSeconds = null;
                sessionStart = DateTime.UtcNow;

                alertLabel.Visible = false;
                alertBeepsRemaining = 0;
                alertTimer.Stop();

                ResetBeepStopState();

                setpointStatusLabel.Text = "Setpoint: -- °C";
                actualStatusLabel.Text = "Actual: -- °C";
                phaseCountdownLabel.Text = "Phase Remaining: --:--";

                ResetXAxisToFullProfile();
            }
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Serial write failed: {ex.Message}");
        }
    }

    private void ToggleHeater()
    {
        if (!serialPort.IsOpen)
        {
            MessageBox.Show("Connect to the Arduino first.");
            return;
        }

        if (heaterManualOn)
        {
            SendCommand("HEATER_OFF");
            SetHeaterManualState(false);
        }
        else
        {
            SendCommand("HEATER_ON");
            SetHeaterManualState(true);
        }
    }

    private void SetHeaterManualState(bool enabled)
    {
        heaterManualOn = enabled;
        heaterToggleButton.Text = enabled ? "Heater: On" : "Heater: Off";
    }

    private void PumpSerial()
    {
        if (!serialPort.IsOpen) return;

        try
        {
            while (serialPort.BytesToRead > 0)
            {
                var line = serialPort.ReadLine().Trim();
                if (line.Length == 0) continue;
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
        var parts = line.Split(';', StringSplitOptions.RemoveEmptyEntries);

        double? temp = null;
        double? setpoint = null;
        double? pt = null;
        string? state = null;
        string? phase = null;

        foreach (var part in parts)
        {
            if (part.StartsWith("T:", StringComparison.OrdinalIgnoreCase) &&
                double.TryParse(part[2..], NumberStyles.Float, CultureInfo.InvariantCulture, out var t))
                temp = t;

            if (part.StartsWith("S:", StringComparison.OrdinalIgnoreCase) &&
                double.TryParse(part[2..], NumberStyles.Float, CultureInfo.InvariantCulture, out var s))
                setpoint = s;

            if (part.StartsWith("STATE:", StringComparison.OrdinalIgnoreCase))
                state = part[6..];

            if (part.StartsWith("PHASE:", StringComparison.OrdinalIgnoreCase))
                phase = part[6..];

            if (part.StartsWith("PT:", StringComparison.OrdinalIgnoreCase) &&
                double.TryParse(part[3..], NumberStyles.Float, CultureInfo.InvariantCulture, out var p))
                pt = p;
        }

        if (!string.IsNullOrWhiteSpace(state))
        {
            currentState = state;
            statusLabel.Text = $"Status: {state}";
        }

        if (!string.IsNullOrWhiteSpace(phase))
        {
            currentPhase = phase;
            phaseLabel.Text = $"Phase: {phase}";
            HighlightCurrentPhase(phase);

            if (phase.Equals("COOL", StringComparison.OrdinalIgnoreCase))
            {
                alertLabel.Visible = true;
                setpointStatusLabel.ForeColor = SystemColors.GrayText;

                if (!beepArmed)
                {
                    beepArmed = true;
                    alertBeepsRemaining = 3;
                    alertTimer.Start();
                }
            }
            else
            {
                alertLabel.Visible = false;
                setpointStatusLabel.ForeColor = SystemColors.ControlText;
            }
        }

        if (pt.HasValue) profileTimeSeconds = pt.Value;

        if (temp.HasValue)
        {
            actualStatusLabel.Text = $"Actual: {temp.Value:0.0} °C";
            UpdateSlopePeakAndBeepStop(temp.Value);
        }

        if (setpoint.HasValue)
            setpointStatusLabel.Text = $"Setpoint: {setpoint.Value:0.0} °C";

        var xSeconds = GetPlotTimeSeconds();
        if (xSeconds.HasValue && temp.HasValue && setpoint.HasValue)
        {
            var xMinutes = ToMinutes(xSeconds.Value);
            chart.Series["Setpoint"].Points.AddXY(xMinutes, setpoint.Value);
            chart.Series["Actual"].Points.AddXY(xMinutes, temp.Value);

            var area = chart.ChartAreas["Reflow"];
            area.AxisX.Minimum = 0;
            var desiredMax = Math.Max(
                ToMinutes(totalProfileSeconds),
                xMinutes + ToMinutes(XAxisBufferSeconds));
            if (area.AxisX.Maximum < desiredMax)
                area.AxisX.Maximum = desiredMax;
        }

        UpdatePhaseCountdown();
    }

    private void UpdateSlopePeakAndBeepStop(double currentTemp)
    {
        // Track peak
        if (currentTemp > peakTempC) peakTempC = currentTemp;

        var now = DateTime.UtcNow;

        if (lastActualTemp is null)
        {
            lastActualTemp = currentTemp;
            lastActualTimeUtc = now;
            filteredSlopeCps = 0.0;
            return;
        }

        var dt = (now - lastActualTimeUtc).TotalSeconds;
        if (dt <= 0.05) return;

        var rawSlope = (currentTemp - lastActualTemp.Value) / dt;
        filteredSlopeCps = filteredSlopeCps + SlopeFilterAlpha * (rawSlope - filteredSlopeCps);

        lastActualTemp = currentTemp;
        lastActualTimeUtc = now;

        // Stop beeping once:
        // 1) We reached reflow (by peak temp), AND
        // 2) Temperature has started going down (negative slope)
        if (beepArmed &&
            peakTempC >= ReflowReachedPeakTempC &&
            filteredSlopeCps <= CoolingSlopeThreshold)
        {
            alertBeepsRemaining = 0;
            alertTimer.Stop();
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

    private double? GetPlotTimeSeconds()
    {
        if (profileTimeSeconds.HasValue) return Math.Max(0, profileTimeSeconds.Value);
        if (sessionStart == DateTime.MinValue) return null;
        return Math.Max(0, (DateTime.UtcNow - sessionStart).TotalSeconds);
    }

    private static double ToMinutes(double seconds)
    {
        return seconds / 60.0;
    }

    private void UpdateCycleTime()
    {
        var elapsed = GetElapsedProfileSeconds();
        if (!elapsed.HasValue)
        {
            elapsedLabel.Text = "Elapsed: --:--";
            remainingLabel.Text = "Remaining: --:--";
            return;
        }

        var elapsedSeconds = (int)Math.Max(0, Math.Min(elapsed.Value, totalProfileSeconds));
        elapsedLabel.Text = $"Elapsed: {FormatSeconds(elapsedSeconds)}";

        var remainingSeconds = Math.Max(0, totalProfileSeconds - elapsedSeconds);
        remainingLabel.Text = $"Remaining: {FormatSeconds(remainingSeconds)}";
        totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
    }

    private double? GetElapsedProfileSeconds()
    {
        if (profileTimeSeconds.HasValue) return profileTimeSeconds.Value;
        if (sessionStart == DateTime.MinValue) return null;
        return (DateTime.UtcNow - sessionStart).TotalSeconds;
    }

    private void UpdatePhaseCountdown()
    {
        if (!profileTimeSeconds.HasValue || !currentState.Equals("RUNNING", StringComparison.OrdinalIgnoreCase))
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
            phaseStart += profileSteps[i].DurationSeconds;

        var phaseEnd = phaseStart + step.DurationSeconds;
        var remainingSeconds = Math.Max(0, phaseEnd - profileTimeSeconds.Value);
        var remaining = (int)Math.Max(0, Math.Ceiling(remainingSeconds));

        phaseCountdownLabel.Text = $"Phase Remaining: {FormatSeconds(remaining)}";
    }

    private bool TryGetProfileStep(string phase, out ProfileStep step, out int index)
    {
        for (int i = 0; i < profileSteps.Count; i++)
        {
            if (profileSteps[i].Label.Equals(phase, StringComparison.OrdinalIgnoreCase))
            {
                step = profileSteps[i];
                index = i;
                return true;
            }
        }

        step = profileSteps[0];
        index = 0;
        return false;
    }

    private void UpdateProfileTotalsAndRender()
    {
        totalProfileSeconds = 0;
        foreach (var step in profileSteps)
            totalProfileSeconds += step.DurationSeconds;

        totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
        RenderProfileSeries();
        ResetXAxisToFullProfile();
    }

    private void LoadProfileRows()
    {
        profileGrid.Rows.Clear();
        profileRowLookup.Clear();
        totalProfileSeconds = 0;

        for (int rowIndex = 0; rowIndex < profileSteps.Count; rowIndex++)
        {
            var step = profileSteps[rowIndex];
            profileGrid.Rows.Add(
                step.Label,
                FormatSeconds(step.DurationSeconds),
                step.StartTempC.ToString("0.0", CultureInfo.InvariantCulture),
                step.EndTempC.ToString("0.0", CultureInfo.InvariantCulture),
                step.OffsetC.ToString("0.0", CultureInfo.InvariantCulture)
            );
            profileRowLookup[step.Label] = rowIndex;
            totalProfileSeconds += step.DurationSeconds;
        }

        totalLabel.Text = $"Total: {FormatSeconds(totalProfileSeconds)}";
        RenderProfileSeries();
        ResetXAxisToFullProfile();
    }

    private static bool TryParseTemperature(string? value, out double temperature)
    {
        return double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out temperature);
    }

    private static bool TryParseDuration(string? value, out int durationSeconds)
    {
        durationSeconds = 0;
        if (string.IsNullOrWhiteSpace(value)) return false;

        if (int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out var seconds))
        {
            durationSeconds = Math.Max(1, seconds);
            return true;
        }

        if (!TimeSpan.TryParseExact(value, @"m\:ss", CultureInfo.InvariantCulture, out var span) &&
            !TimeSpan.TryParseExact(value, @"mm\:ss", CultureInfo.InvariantCulture, out span))
            return false;

        durationSeconds = Math.Max(1, (int)Math.Round(span.TotalSeconds));
        return true;
    }

    private void SaveProfile()
    {
        using var dialog = new SaveFileDialog
        {
            Filter = "Reflow Profile (*.json)|*.json",
            DefaultExt = "json",
            AddExtension = true,
            FileName = $"reflow-profile-{DateTime.Now:yyyyMMdd-HHmmss}.json"
        };

        if (dialog.ShowDialog() != DialogResult.OK) return;

        try
        {
            var payload = new ProfilePayload { Steps = profileSteps };
            var json = JsonSerializer.Serialize(payload, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(dialog.FileName, json);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to save profile: {ex.Message}");
        }
    }

    private void LoadProfile()
    {
        using var dialog = new OpenFileDialog
        {
            Filter = "Reflow Profile (*.json)|*.json",
            DefaultExt = "json"
        };

        if (dialog.ShowDialog() != DialogResult.OK) return;

        try
        {
            var json = File.ReadAllText(dialog.FileName);
            var payload = JsonSerializer.Deserialize<ProfilePayload>(json);
            if (payload?.Steps is null || payload.Steps.Count == 0)
            {
                MessageBox.Show("Profile file did not contain any steps.");
                return;
            }

            if (!TryApplyProfile(payload.Steps))
                MessageBox.Show("Profile file did not match the expected phases.");
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to load profile: {ex.Message}");
        }
    }

    private void ResetProfileToDefault()
    {
        profileSteps.Clear();
        profileSteps.AddRange(DefaultProfileSteps);
        LoadProfileRows();
    }

    private bool TryApplyProfile(IReadOnlyList<ProfileStep> steps)
    {
        var stepsByLabel = new Dictionary<string, ProfileStep>(StringComparer.OrdinalIgnoreCase);
        foreach (var step in steps)
            stepsByLabel[step.Label] = step;

        for (int i = 0; i < profileSteps.Count; i++)
        {
            var label = profileSteps[i].Label;
            if (!stepsByLabel.TryGetValue(label, out var replacement))
                return false;
            profileSteps[i] = replacement with { Label = label };
        }

        LoadProfileRows();
        return true;
    }

    private sealed class ProfilePayload
    {
        public List<ProfileStep> Steps { get; init; } = new();
    }

    private void SaveChartImage()
    {
        using var dialog = new SaveFileDialog
        {
            Filter = "PNG Image (*.png)|*.png",
            DefaultExt = "png",
            AddExtension = true,
            FileName = $"reflow-{DateTime.Now:yyyyMMdd-HHmmss}.png"
        };

        if (dialog.ShowDialog() != DialogResult.OK) return;

        try
        {
            chart.SaveImage(dialog.FileName, ChartImageFormat.Png);
        }
        catch (Exception ex)
        {
            MessageBox.Show($"Failed to save image: {ex.Message}");
        }
    }

    private void SaveLastPort(string portName)
    {
        lastPortName = portName;
        try
        {
            var directory = Path.GetDirectoryName(settingsFilePath);
            if (!string.IsNullOrWhiteSpace(directory))
                Directory.CreateDirectory(directory);
            File.WriteAllText(settingsFilePath, portName);
        }
        catch
        {
            // ignore settings persistence errors
        }
    }

    private void LoadSettings()
    {
        try
        {
            if (File.Exists(settingsFilePath))
                lastPortName = File.ReadAllText(settingsFilePath).Trim();
        }
        catch
        {
            lastPortName = null;
        }
    }
}
