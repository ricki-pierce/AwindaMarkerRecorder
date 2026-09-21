using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Windows.Forms;
using XDA;

namespace AwindaMarkerRecorder
{
    /// <summary>
    /// Tracks which MTw sensors are currently connected to the Awinda station.
    /// </summary>
    public class WirelessMasterCallback : XsCallback
    {
        // This SDK version passes the device as the opaque SWIGTYPE_p_XsDevice
        // pointer type rather than a full XsDevice, and doesn't reliably let us
        // key a Set by device identity here -- so we track a simple connected
        // count instead. It's only used to know when at least one sensor has
        // joined; the actual sensor list is re-read from XsControl afterward.
        private int _connectedCount = 0;
        private readonly object _lock = new object();

        public int GetConnectedCount()
        {
            lock (_lock) { return _connectedCount; }
        }

        protected override void onConnectivityChanged(SWIGTYPE_p_XsDevice dev, XsConnectivityState newState)
        {
            lock (_lock)
            {
                if (newState == XsConnectivityState.XCS_Wireless)
                    _connectedCount++;
                else
                    _connectedCount = Math.Max(0, _connectedCount - 1);
            }
        }
    }

    /// <summary>
    /// Buffers incoming packets for a single MTw sensor.
    /// </summary>
    public class MtwCallback : XsCallback
    {
        public int Index { get; }
        public XsDevice Device { get; }

        // Cached once so the CSV gets a real sensor id instead of "XDA.XsDeviceId".
        public string IdString { get; }

        private readonly Queue<XsDataPacket> _buffer = new Queue<XsDataPacket>();
        private readonly object _lock = new object();
        private const int MaxBuffer = 300;

        public MtwCallback(int index, XsDevice device)
        {
            Index = index;
            Device = device;
            try { IdString = device.deviceId().toXsString().toString(); }
            catch { IdString = "sensor" + index; }
        }

        public bool DataAvailable()
        {
            lock (_lock) { return _buffer.Count > 0; }
        }

        public XsDataPacket GetOldest()
        {
            lock (_lock) { return _buffer.Count > 0 ? _buffer.Peek() : null; }
        }

        public void PopOldest()
        {
            lock (_lock) { if (_buffer.Count > 0) _buffer.Dequeue(); }
        }

        protected override void onDataAvailable(SWIGTYPE_p_XsDevice dev, XsDataPacket packet)
        {
            // Copy the packet (as the reference MyXda/MyMtwCallback example does)
            // since we process it later, outside this callback's lifetime.
            var packetCopy = new XsDataPacket(packet);
            lock (_lock)
            {
                _buffer.Enqueue(packetCopy);
                while (_buffer.Count > MaxBuffer) _buffer.Dequeue();
            }
        }
    }

    /// <summary>
    /// Wraps the Xsens Device API session: connect, arm, record, mark events, stop.
    ///
    /// Split into Prepare() + Go() so that all the slow work (device config,
    /// gotoMeasurement, log-file creation, waiting for the first packets) happens
    /// BEFORE the trial starts. Go() then only calls startRecording(), which lets
    /// the Python master start the IMUs and the AMTI force plate within ~1 ms of
    /// each other when the operator hits SPACEBAR.
    /// </summary>
    public class XsensRecorder
    {
        private const int DesiredUpdateRate = 60;      // Hz
        private const int DesiredRadioChannel = 11;     // 11-25

        // Settable from Python via the SETDIR command.
        public string LogDir { get; set; } = @"C:\AMTI\PythonResults\IMU";

        public Action<string> Log;

        private XsControl _control;
        private XsDevice _master;
        private readonly WirelessMasterCallback _masterCb = new WirelessMasterCallback();
        private readonly List<XsDevice> _mtwDevices = new List<XsDevice>();
        private readonly List<MtwCallback> _mtwCallbacks = new List<MtwCallback>();

        private volatile bool _recording;
        private volatile bool _prepared;
        private volatile bool _stopRequested;
        private Thread _dataThread;
        private double _recordingStartUnix;

        private readonly object _eventLock = new object();
        // (unix time, marker code, label)
        private readonly Queue<Tuple<double, int, string>> _eventQueue =
            new Queue<Tuple<double, int, string>>();

        private StreamWriter _csvWriter;
        private StreamWriter _eventsWriter;
        private string _mtbPath, _csvPath, _eventsPath;

        public bool IsRecording => _recording;
        public bool IsPrepared => _prepared;
        public bool IsConnected => _master != null;

        private static double NowUnix() =>
            (DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalSeconds;

        // -- connection -------------------------------------------------
        public void Connect()
        {
            if (_master != null) { Log?.Invoke("Already connected."); return; }

            Log?.Invoke("Constructing XsControl...");
            _control = new XsControl();
            if (_control == null) throw new Exception("Failed to construct XsControl");

            Log?.Invoke("Scanning ports for the Awinda station...");
            XsPortInfoArray ports = XsScanner.scanPorts();

            XsPortInfo masterPort = null;
            for (uint i = 0; i < ports.size(); i++)
            {
                XsPortInfo p = ports.at(i);
                if (p.deviceId().isWirelessMaster()) { masterPort = p; break; }
            }
            if (masterPort == null) throw new Exception("No Awinda station found. Check the USB dongle.");

            Log?.Invoke($"Opening port {masterPort.portName()}...");
            if (!_control.openPort(masterPort))
                throw new Exception("Failed to open port");

            _master = new XsDevice(_control.device(masterPort.deviceId()));
            if (_master == null) throw new Exception("Failed to get master device instance");

            _master.gotoConfig();
            _master.addCallbackHandler(_masterCb);

            // Some SDK builds want XsDevice.supportedUpdateRates(device, XsDataIdentifier.XDI_None);
            // if this line doesn't compile, check the equivalent call in your own
            // awindamonitor_csharp / MyXda.cs example and swap it in here.
            _master.setUpdateRate(DesiredUpdateRate);

            if (_master.isRadioEnabled())
                _master.disableRadio();
            if (!_master.enableRadio(DesiredRadioChannel))
                throw new Exception("Failed to enable radio");

            Log?.Invoke($"Radio enabled on channel {DesiredRadioChannel} @ {DesiredUpdateRate} Hz.");
            Log?.Invoke("Waiting for MTw sensor(s) to connect (power on / undock them)...");
        }

        public int WaitForSensors(int minCount = 1, int timeoutSeconds = 30)
        {
            var start = DateTime.UtcNow;
            int lastN = -1;
            while ((DateTime.UtcNow - start).TotalSeconds < timeoutSeconds)
            {
                int n = _masterCb.GetConnectedCount();
                if (n != lastN)
                {
                    Log?.Invoke($"{n} sensor(s) connected");
                    lastN = n;
                }
                if (n >= minCount) return n;
                Thread.Sleep(200);
            }
            return _masterCb.GetConnectedCount();
        }

        // -- arming (slow half of the old StartRecording) ------------------------
        /// <summary>
        /// Configure the sensors, create the log files for this condition, and wait
        /// until every sensor is streaming. Does NOT start recording -- call Go().
        /// </summary>
        public void Prepare(string sessionName)
        {
            if (_master == null) throw new Exception("Not connected. Send CONNECT first.");
            if (_recording) throw new Exception("Already recording. Send STOP first.");
            if (_prepared) { Log?.Invoke("Already prepared."); return; }

            // Discover MTw devices WHILE STILL IN CONFIG MODE, not after gotoMeasurement()
            var allIds = _control.deviceIds();
            _mtwDevices.Clear();
            _mtwCallbacks.Clear();

            for (uint i = 0; i < allIds.size(); i++)
            {
                XsDeviceId id = allIds.at(i);
                if (id.isMtw())
                {
                    var dev = new XsDevice(_control.device(id));
                    if (dev != null) _mtwDevices.Add(dev);
                }
            }

            if (_mtwDevices.Count == 0)
                throw new Exception("No MTw sensors found. Connect at least one before starting.");

            Log?.Invoke($"Configuring output for {_mtwDevices.Count} sensor(s)...");

            // Explicitly request packet counter, sample time, and orientation (Euler)
            // from every sensor -- don't rely on factory defaults.
            foreach (var dev in _mtwDevices)
            {
                var configArray = new XsOutputConfigurationArray();
                configArray.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_PacketCounter, 0));
                configArray.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_SampleTimeFine, 0));
                configArray.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_EulerAngles, (ushort)DesiredUpdateRate));

                if (!dev.setOutputConfiguration(configArray))
                    throw new Exception($"Failed to set output configuration on device {dev.deviceId().toXsString().toString()}");
            }

            // NOW go to measurement mode, after configuration is set on every sensor
            _master.gotoMeasurement();

            for (int i = 0; i < _mtwDevices.Count; i++)
            {
                var cb = new MtwCallback(i, _mtwDevices[i]);
                _mtwCallbacks.Add(cb);
                _mtwDevices[i].addCallbackHandler(cb);
            }

            // ---- file names come from Python so they match the force-plate files ----
            if (string.IsNullOrWhiteSpace(sessionName))
                sessionName = "session_" + DateTime.Now.ToString("yyyyMMdd_HHmmss");
            foreach (char c in Path.GetInvalidFileNameChars())
                sessionName = sessionName.Replace(c, '_');

            Directory.CreateDirectory(LogDir);
            _mtbPath    = Path.Combine(LogDir, sessionName + ".mtb");
            _csvPath    = Path.Combine(LogDir, sessionName + ".csv");
            _eventsPath = Path.Combine(LogDir, sessionName + "_events.csv");

            if (_master.createLogFile(new XsString(_mtbPath)) != XsResultValue.XRV_OK)
                throw new Exception("Failed to create .mtb log file: " + _mtbPath);

            _csvWriter = new StreamWriter(_csvPath, false);
            _csvWriter.WriteLine("sensor_id,t_unix,packet_counter,roll_deg,pitch_deg,yaw_deg");

            _eventsWriter = new StreamWriter(_eventsPath, false);
            _eventsWriter.WriteLine("t_unix,t_iso,marker,marker_label");

            // Wait until every sensor is actually delivering packets (with a timeout,
            // so a dropped sensor can't hang the Python master forever).
            var sw = Stopwatch.StartNew();
            bool ready = false;
            while (!ready)
            {
                ready = true;
                foreach (var cb in _mtwCallbacks)
                    if (!cb.DataAvailable()) { ready = false; break; }
                if (ready) break;
                if (sw.Elapsed.TotalSeconds > 15)
                    throw new Exception("Timed out waiting for sensor data. Check that all MTw sensors are awake.");
                Thread.Sleep(50);
            }

            _prepared = true;
            Log?.Invoke($"Armed -> {_mtbPath}");
        }

        // -- recording (fast half) -----------------------------------------------
        /// <summary>
        /// Actually start recording. Returns the unix timestamp of the start so the
        /// Python side can verify alignment with the force plate.
        /// </summary>
        public double Go()
        {
            if (!_prepared) throw new Exception("Not prepared. Send PREPARE first.");
            if (_recording) return _recordingStartUnix;

            if (!_master.startRecording())
                throw new Exception("Failed to start recording");

            _recordingStartUnix = NowUnix();
            _recording = true;
            _stopRequested = false;

            lock (_eventLock)
                _eventQueue.Enqueue(Tuple.Create(_recordingStartUnix, 0, "recording_start"));

            _dataThread = new Thread(DataLoop) { IsBackground = true };
            _dataThread.Start();
            Log?.Invoke($"Recording started @ {_recordingStartUnix:F6} -> {_mtbPath}");
            return _recordingStartUnix;
        }

        private void DataLoop()
        {
            var flushTimer = Stopwatch.StartNew();

            while (!_stopRequested)
            {
                foreach (var cb in _mtwCallbacks)
                {
                    if (cb.DataAvailable())
                    {
                        var packet = cb.GetOldest();
                        if (packet != null)
                        {
                            var euler = packet.orientationEuler();
                            _csvWriter.WriteLine(
                                $"{cb.IdString},{NowUnix():F6},{packet.packetCounter()}," +
                                $"{euler.x():F3},{euler.y():F3},{euler.z():F3}");
                        }
                        cb.PopOldest();
                    }
                }

                lock (_eventLock)
                {
                    while (_eventQueue.Count > 0)
                    {
                        var evt = _eventQueue.Dequeue();
                        var iso = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)
                            .AddSeconds(evt.Item1).ToLocalTime().ToString("o");
                        // quote the label: it can contain commas (e.g. "ONLY_BLUE 1,2,3,0")
                        _eventsWriter.WriteLine(
                            $"{evt.Item1:F6},{iso},{evt.Item2},\"{evt.Item3.Replace("\"", "'")}\"");
                        _eventsWriter.Flush();
                    }
                }

                if (flushTimer.ElapsedMilliseconds > 1000)
                {
                    _csvWriter.Flush();
                    flushTimer.Restart();
                }

                Thread.Sleep(1);
            }
        }

        /// <summary>Marker with a numeric code (matches the Python LSL / force-plate codes).</summary>
        public double Mark(int code, string label)
        {
            if (!_recording)
            {
                Log?.Invoke($"Not recording -- marker '{label}' ignored.");
                return 0.0;
            }
            double t = NowUnix();
            lock (_eventLock) { _eventQueue.Enqueue(Tuple.Create(t, code, label ?? "")); }
            Log?.Invoke($"Marker {code} '{label}' @ {t:F3}");
            return t;
        }

        /// <summary>Kept so the on-screen placeholder buttons still work.</summary>
        public void Mark(string label) { Mark(0, label); }

        public void StopRecording()
        {
            if (!_recording && !_prepared) return;

            if (_recording)
            {
                _stopRequested = true;
                _dataThread?.Join(2000);
                _master.stopRecording();
            }

            _master.closeLogFile();

            // Detach this run's callbacks before going back to config, otherwise every
            // condition leaves another live handler attached to each sensor.
            for (int i = 0; i < _mtwCallbacks.Count && i < _mtwDevices.Count; i++)
            {
                try { _mtwDevices[i].removeCallbackHandler(_mtwCallbacks[i]); } catch { }
            }
            _mtwCallbacks.Clear();
            _mtwDevices.Clear();

            _master.gotoConfig();

            _csvWriter?.Flush();
            _csvWriter?.Close();
            _eventsWriter?.Flush();
            _eventsWriter?.Close();
            _csvWriter = null;
            _eventsWriter = null;

            _recording = false;
            _prepared = false;
            Log?.Invoke($"Recording stopped. Files: {_csvPath}");
        }

        public void Close()
        {
            try
            {
                if (_recording || _prepared) StopRecording();
                if (_master != null && _master.isRadioEnabled()) _master.disableRadio();
                _control?.close();
                _master = null;
            }
            catch (Exception e)
            {
                Log?.Invoke($"Error during shutdown: {e.Message}");
            }
        }
    }

    /// <summary>
    /// Main window. Normally driven by the Python master script over the command
    /// server; the manual buttons are kept only for standalone testing.
    /// </summary>
    public class RecorderForm : Form
    {
        // Must match IMU_PORT in the Python master script.
        private const int CommandPort = 5555;

        private readonly XsensRecorder _recorder = new XsensRecorder();
        private CommandServer _server;

        private TextBox _log;
        private Button _startBtn;
        private Button _litBtn;
        private Button _pressBtn;
        private Button _stopBtn;
        private NumericUpDown _sensorCountInput;

        public RecorderForm()
        {
            Text = "Xsens Awinda Recorder (driven by Python master)";
            Width = 640;
            Height = 480;

            _log = new TextBox
            {
                Multiline = true,
                ScrollBars = ScrollBars.Vertical,
                ReadOnly = true,
                Dock = DockStyle.Top,
                Height = 320
            };
            Controls.Add(_log);

            var panel = new FlowLayoutPanel { Dock = DockStyle.Bottom, Height = 60 };
            Controls.Add(panel);

            panel.Controls.Add(new Label { Text = "Sensors:", AutoSize = true, Padding = new Padding(5, 12, 0, 0) });
            _sensorCountInput = new NumericUpDown { Minimum = 1, Maximum = 20, Value = 2, Width = 50, Margin = new Padding(3, 12, 10, 3) };
            panel.Controls.Add(_sensorCountInput);

            // --- Manual test buttons. Do NOT touch these during a real session; the
            // --- Python script owns connect/arm/start/mark/stop.
            _startBtn = new Button { Text = "Manual Start", Width = 120, Height = 40 };
            _startBtn.Click += (s, e) => OnStart();
            panel.Controls.Add(_startBtn);

            _litBtn = new Button { Text = "Button Lit", Width = 120, Height = 40, Enabled = false };
            _litBtn.Click += (s, e) => _recorder.Mark(2, "manual button_lit");
            panel.Controls.Add(_litBtn);

            _pressBtn = new Button { Text = "Button Press", Width = 120, Height = 40, Enabled = false };
            _pressBtn.Click += (s, e) => _recorder.Mark(4, "manual button_press");
            panel.Controls.Add(_pressBtn);

            _stopBtn = new Button { Text = "Manual Stop", Width = 120, Height = 40, Enabled = false };
            _stopBtn.Click += (s, e) => OnStop();
            panel.Controls.Add(_stopBtn);

            _recorder.Log = AppendLog;

            _server = new CommandServer(_recorder, CommandPort) { Log = AppendLog };
            try { _server.Start(); }
            catch (Exception ex) { AppendLog("Could not start command server: " + ex.Message); }

            FormClosing += (s, e) =>
            {
                _server?.Stop();
                _recorder.Close();
            };

            AppendLog("Ready. Leave this window open and start the Python master script.");
        }

        private void AppendLog(string msg)
        {
            if (InvokeRequired) { BeginInvoke(new Action<string>(AppendLog), msg); return; }
            _log.AppendText(msg + Environment.NewLine);
            // keep the log from growing without bound over a long session
            if (_log.TextLength > 200000) _log.Clear();
        }

        private void OnStart()
        {
            _startBtn.Enabled = false;
            int expectedSensors = (int)_sensorCountInput.Value;
            var t = new Thread(() =>
            {
                try
                {
                    _recorder.Connect();
                    _recorder.WaitForSensors(expectedSensors, 30);
                    _recorder.Prepare("manual_test");
                    _recorder.Go();
                    BeginInvoke(new Action(() =>
                    {
                        _litBtn.Enabled = true;
                        _pressBtn.Enabled = true;
                        _stopBtn.Enabled = true;
                    }));
                }
                catch (Exception ex)
                {
                    AppendLog("ERROR: " + ex.Message);
                    BeginInvoke(new Action(() => _startBtn.Enabled = true));
                }
            })
            { IsBackground = true };
            t.Start();
        }

        private void OnStop()
        {
            _stopBtn.Enabled = false;
            var t = new Thread(() =>
            {
                _recorder.StopRecording();
                BeginInvoke(new Action(() =>
                {
                    _startBtn.Enabled = true;
                    _litBtn.Enabled = false;
                    _pressBtn.Enabled = false;
                }));
            })
            { IsBackground = true };
            t.Start();
        }
    }
}
