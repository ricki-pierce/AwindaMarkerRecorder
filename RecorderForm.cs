using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Windows.Forms;
using XDA;

namespace AwindaMarkerRecorder
{
    /// <summary>
    /// High-resolution UTC clock. DateTime.UtcNow can tick as coarsely as ~15 ms on .NET Framework,
    /// which is about one IMU packet at 60 Hz. This anchors to UtcNow once, then advances with a Stopwatch.
    /// Used for BOTH packet arrival times and marker times so they line up.
    /// </summary>
    internal static class PreciseClock
    {
        private static readonly DateTime _baseUtc = DateTime.UtcNow;
        private static readonly Stopwatch _sw = Stopwatch.StartNew();
        private static readonly DateTime _epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        public static DateTime UtcNow { get { return _baseUtc + _sw.Elapsed; } }
        public static double ToUnix(DateTime utc) { return (utc - _epoch).TotalSeconds; }
    }

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

    /// <summary>A packet plus the precise moment it arrived at the PC.</summary>
    public class PacketItem
    {
        public XsDataPacket Packet;
        public DateTime ArrivalUtc;
    }

    /// <summary>
    /// Buffers incoming packets for a single MTw sensor.
    /// </summary>
    public class MtwCallback : XsCallback
    {
        public int Index { get; }
        public XsDevice Device { get; }
        public string SensorId { get; }

        private readonly Queue<PacketItem> _buffer = new Queue<PacketItem>();
        private readonly object _lock = new object();
        private const int MaxBuffer = 600;

        public MtwCallback(int index, XsDevice device)
        {
            Index = index;
            Device = device;
            SensorId = device.deviceId().ToString();
        }

        public bool DataAvailable()
        {
            lock (_lock) { return _buffer.Count > 0; }
        }

        public PacketItem GetOldest()
        {
            lock (_lock) { return _buffer.Count > 0 ? _buffer.Peek() : null; }
        }

        public void PopOldest()
        {
            lock (_lock) { if (_buffer.Count > 0) _buffer.Dequeue(); }
        }

        protected override void onDataAvailable(SWIGTYPE_p_XsDevice dev, XsDataPacket packet)
        {
            // Stamp arrival time FIRST (this is the time used for marker alignment), then copy the
            // packet (as the reference MyXda/MyMtwCallback example does).
            var item = new PacketItem
            {
                ArrivalUtc = PreciseClock.UtcNow,
                Packet = new XsDataPacket(packet)
            };
            lock (_lock)
            {
                _buffer.Enqueue(item);
                while (_buffer.Count > MaxBuffer) _buffer.Dequeue();
            }
        }
    }

    internal class EventItem
    {
        public DateTime T;
        public int Code;
        public string Label;
        public string Condition;
        public double PythonUnix;
    }

    /// <summary>
    /// Wraps the Xsens Device API session: connect, record, mark events, stop.
    /// </summary>
    public class XsensRecorder
    {
        private const int DesiredUpdateRate = 60;      // Hz
        private const int DesiredRadioChannel = 11;     // 11-25
        private const string LogDir = @"C:\Users\rpier12\Documents\IMUData";

        private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        private const string CsvHeader =
            "sensor_id,t_unix,pc_datetime_local,pc_datetime_utc,sensor_utc,packet_counter,sample_time_fine,sample_time_coarse,status_word," +
            "quat_w,quat_x,quat_y,quat_z,roll_deg,pitch_deg,yaw_deg," +
            "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,pressure_pa," +
            "free_acc_x,free_acc_y,free_acc_z,delta_q_w,delta_q_x,delta_q_y,delta_q_z,delta_v_x,delta_v_y,delta_v_z," +
            "battery_pct,rssi,marker_code,marker_label,condition";

        public Action<string> Log;

        public string CsvPath { get; private set; }
        public string EventsPath { get; private set; }
        public string MtbPath { get; private set; }

        private XsControl _control;
        private XsDevice _master;
        private readonly WirelessMasterCallback _masterCb = new WirelessMasterCallback();
        private readonly List<XsDevice> _mtwDevices = new List<XsDevice>();
        private readonly List<MtwCallback> _mtwCallbacks = new List<MtwCallback>();

        private volatile bool _recording;
        private volatile bool _stopRequested;
        private volatile string _condition = "";
        private Thread _dataThread;

        private readonly object _eventLock = new object();
        private readonly Queue<EventItem> _eventQueue = new Queue<EventItem>();
        // per-sensor markers waiting to be stamped onto that sensor's NEXT data row
        private readonly Dictionary<string, List<KeyValuePair<int, string>>> _pendingMarks =
            new Dictionary<string, List<KeyValuePair<int, string>>>();

        private readonly HashSet<string> _reported = new HashSet<string>();
        private readonly Dictionary<string, string[]> _linkInfo = new Dictionary<string, string[]>();
        private readonly Dictionary<string, double> _linkInfoTime = new Dictionary<string, double>();
        private readonly Stopwatch _linkWatch = Stopwatch.StartNew();

        private StreamWriter _csvWriter;
        private StreamWriter _eventsWriter;

        public bool IsRecording { get { return _recording; } }

        // -- connection -------------------------------------------------
        public void Connect()
        {
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

        // Tier 1 = everything an MTw can send. If the sensor rejects it, fall back to smaller sets.
        private static XsOutputConfigurationArray BuildConfig(int tier)
        {
            ushort rate = (ushort)DesiredUpdateRate;
            var a = new XsOutputConfigurationArray();
            a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_PacketCounter, 0));
            a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_SampleTimeFine, 0));
            a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_EulerAngles, rate));
            if (tier <= 2)
            {
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_StatusWord, 0));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_Quaternion, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_Acceleration, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_RateOfTurn, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_MagneticField, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_FreeAcceleration, rate));
            }
            if (tier <= 1)
            {
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_BaroPressure, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_DeltaQ, rate));
                a.push_back(new XsOutputConfiguration(XsDataIdentifier.XDI_DeltaV, rate));
            }
            return a;
        }

        // -- recording ----------------------------------------------------------
        public void StartRecording()
        {
            if (_recording) return;

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

            // Request as many data types as the sensor will accept (see BuildConfig).
            foreach (var dev in _mtwDevices)
            {
                bool ok = false;
                for (int tier = 1; tier <= 3 && !ok; tier++)
                {
                    if (dev.setOutputConfiguration(BuildConfig(tier)))
                    {
                        ok = true;
                        Log?.Invoke($"Sensor {dev.deviceId()}: output configuration tier {tier} accepted " +
                                    "(1 = all data, 2 = no pressure/delta, 3 = basic).");
                    }
                }
                if (!ok)
                    throw new Exception($"Failed to set output configuration on device {dev.deviceId()}");
            }

            // NOW go to measurement mode, after configuration is set on every sensor
            _master.gotoMeasurement();

            for (int i = 0; i < _mtwDevices.Count; i++)
            {
                var cb = new MtwCallback(i, _mtwDevices[i]);
                _mtwCallbacks.Add(cb);
                _mtwDevices[i].addCallbackHandler(cb);
            }

            lock (_eventLock)
            {
                _pendingMarks.Clear();
                foreach (var cb in _mtwCallbacks)
                    _pendingMarks[cb.SensorId] = new List<KeyValuePair<int, string>>();
                _eventQueue.Clear();
            }
            _reported.Clear();
            _linkInfo.Clear();
            _linkInfoTime.Clear();

            Directory.CreateDirectory(LogDir);
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            MtbPath = Path.Combine(LogDir, $"session_{stamp}.mtb");
            CsvPath = Path.Combine(LogDir, $"session_{stamp}.csv");
            EventsPath = Path.Combine(LogDir, $"session_{stamp}_events.csv");

            if (_master.createLogFile(new XsString(MtbPath)) != XsResultValue.XRV_OK)
                throw new Exception("Failed to create .mtb log file");

            _csvWriter = new StreamWriter(CsvPath, false);
            _csvWriter.WriteLine(CsvHeader);

            _eventsWriter = new StreamWriter(EventsPath, false);
            _eventsWriter.WriteLine("t_unix,t_iso,marker_code,label,condition,python_t_unix");

            // Wait until every sensor is actually delivering packets (30 s limit)
            var readyStart = DateTime.UtcNow;
            bool ready = false;
            while (!ready)
            {
                ready = true;
                foreach (var cb in _mtwCallbacks)
                    if (!cb.DataAvailable()) { ready = false; break; }
                if (!ready && (DateTime.UtcNow - readyStart).TotalSeconds > 30)
                    throw new Exception("Sensors never started delivering packets.");
                Thread.Sleep(50);
            }

            if (!_master.startRecording())
                throw new Exception("Failed to start recording");

            _recording = true;
            _stopRequested = false;
            _dataThread = new Thread(DataLoop) { IsBackground = true };
            _dataThread.Start();
            Log?.Invoke($"Recording started -> {MtbPath}");
            Log?.Invoke("NOTE: the MTw filter needs ~1 minute of calm, slow movement to settle.");
        }

        // ---- helpers for building CSV rows ----
        private static void D(List<string> c, double v) { c.Add(double.IsNaN(v) ? "" : v.ToString("R", Inv)); }
        private static void Vec3(List<string> c, XsVector v) { for (uint i = 0; i < 3; i++) D(c, v.value(i)); }
        private static void Blank(List<string> c, int n) { for (int i = 0; i < n; i++) c.Add(""); }
        private static string Clean(string s) { return (s ?? "").Replace(",", ";").Replace("\r", " ").Replace("\n", " "); }

        private string[] GetLinkInfo(MtwCallback cb)
        {
            double now = _linkWatch.Elapsed.TotalSeconds;
            string[] info;
            if (!_linkInfo.TryGetValue(cb.SensorId, out info) || now - _linkInfoTime[cb.SensorId] > 1.0)
            {
                string b = "";
                try { b = cb.Device.batteryLevel().ToString(); } catch { }
                info = new[] { b };
                _linkInfo[cb.SensorId] = info;
                _linkInfoTime[cb.SensorId] = now;
            }
            return info;
        }

        private static string Describe(XsDataPacket p)
        {
            var s = new List<string>();
            if (p.containsPacketCounter()) s.Add("packetCounter");
            if (p.containsSampleTimeFine()) s.Add("sampleTimeFine");
            if (p.containsSampleTimeCoarse()) s.Add("sampleTimeCoarse");
            if (p.containsStatus()) s.Add("status");
            if (p.containsUtcTime()) s.Add("utcTime");
            if (p.containsOrientation()) s.Add("orientation");
            if (p.containsCalibratedAcceleration()) s.Add("acceleration");
            if (p.containsCalibratedGyroscopeData()) s.Add("gyroscope");
            if (p.containsCalibratedMagneticField()) s.Add("magnetometer");
            if (p.containsPressure()) s.Add("pressure");
            if (p.containsFreeAcceleration()) s.Add("freeAcceleration");
            if (p.containsSdiData()) s.Add("sdiData(deltaQ/deltaV)");
            return string.Join(", ", s.ToArray());
        }

        private void WriteRow(MtwCallback cb, PacketItem item)
        {
            XsDataPacket p = item.Packet;
            DateTime utc = item.ArrivalUtc;
            var c = new List<string>(48);

            if (_reported.Add(cb.SensorId))
                Log?.Invoke($"Sensor {cb.SensorId} packets contain: {Describe(p)}");

            // GROUP: identity + PC time (arrival time, high resolution)
            c.Add(cb.SensorId);
            c.Add(PreciseClock.ToUnix(utc).ToString("F6", Inv));
            c.Add(utc.ToLocalTime().ToString("yyyy-MM-dd HH:mm:ss.ffffff", Inv));
            c.Add(utc.ToString("yyyy-MM-dd HH:mm:ss.ffffff", Inv));

            // GROUP: sensor UTC (MTw often doesn't provide it -> blank)
            if (p.containsUtcTime())
            {
                var u = p.utcTime();
                c.Add(string.Format(Inv, "{0:D4}-{1:D2}-{2:D2} {3:D2}:{4:D2}:{5:D2}.{6:D3}",
                    u.m_year, u.m_month, u.m_day, u.m_hour, u.m_minute, u.m_second, u.m_nano / 1000000));
            }
            else c.Add("");

            // GROUP: counters + status
            c.Add(p.containsPacketCounter() ? p.packetCounter().ToString() : "");
            c.Add(p.containsSampleTimeFine() ? p.sampleTimeFine().ToString() : "");
            c.Add(p.containsSampleTimeCoarse() ? p.sampleTimeCoarse().ToString() : "");
            c.Add(p.containsStatus() ? p.status().ToString() : "");

            // GROUP: orientation (quaternion + Euler)
            if (p.containsOrientation())
            {
                XsQuaternion q = p.orientationQuaternion();
                D(c, q.w()); D(c, q.x()); D(c, q.y()); D(c, q.z());
                XsEuler e = p.orientationEuler();
                D(c, e.x()); D(c, e.y()); D(c, e.z());
            }
            else Blank(c, 7);

            // GROUP: acceleration
            if (p.containsCalibratedAcceleration()) Vec3(c, p.calibratedAcceleration()); else Blank(c, 3);
            // GROUP: gyroscope (angular velocity)
            if (p.containsCalibratedGyroscopeData()) Vec3(c, p.calibratedGyroscopeData()); else Blank(c, 3);
            // GROUP: magnetometer
            if (p.containsCalibratedMagneticField()) Vec3(c, p.calibratedMagneticField()); else Blank(c, 3);
            // GROUP: pressure
            if (p.containsPressure()) D(c, p.pressure().m_pressure); else Blank(c, 1);
            // GROUP: free acceleration
            if (p.containsFreeAcceleration()) Vec3(c, p.freeAcceleration()); else Blank(c, 3);

            // GROUP: strapdown-integration increments
            if (p.containsSdiData())
            {
                XsSdiData s = p.sdiData();
                XsQuaternion dq = s.orientationIncrement();
                XsVector dv = s.velocityIncrement();
                D(c, dq.w()); D(c, dq.x()); D(c, dq.y()); D(c, dq.z());
                D(c, dv.value(0)); D(c, dv.value(1)); D(c, dv.value(2));
            }
            else Blank(c, 7);

            // GROUP: link info (battery, RSSI; refreshed once per second)
            string[] link = GetLinkInfo(cb);
            c.Add(link[0]);
            c.Add(p.containsRssi() ? p.rssi().ToString() : "");

            // GROUP: markers stamped onto this sensor's next packet
            string mc = "", ml = "";
            lock (_eventLock)
            {
                List<KeyValuePair<int, string>> pend;
                if (_pendingMarks.TryGetValue(cb.SensorId, out pend) && pend.Count > 0)
                {
                    var codes = new List<string>(); var labels = new List<string>();
                    foreach (var m in pend) { codes.Add(m.Key.ToString()); labels.Add(Clean(m.Value)); }
                    mc = string.Join(";", codes.ToArray());
                    ml = string.Join(" | ", labels.ToArray());
                    pend.Clear();
                }
            }
            c.Add(mc); c.Add(ml); c.Add(Clean(_condition));

            _csvWriter.WriteLine(string.Join(",", c.ToArray()));
        }

        private void DrainEvents()
        {
            lock (_eventLock)
            {
                while (_eventQueue.Count > 0)
                {
                    var evt = _eventQueue.Dequeue();
                    string py = double.IsNaN(evt.PythonUnix) ? "" : evt.PythonUnix.ToString("F6", Inv);
                    _eventsWriter.WriteLine(string.Format(Inv, "{0:F6},{1},{2},{3},{4},{5}",
                        PreciseClock.ToUnix(evt.T), evt.T.ToLocalTime().ToString("o"),
                        evt.Code, Clean(evt.Label), Clean(evt.Condition), py));
                }
                _eventsWriter.Flush();
            }
        }

        private void DataLoop()
        {
            var flushWatch = Stopwatch.StartNew();
            while (!_stopRequested)
            {
                foreach (var cb in _mtwCallbacks)
                {
                    PacketItem item = cb.GetOldest();
                    if (item != null)
                    {
                        try { WriteRow(cb, item); }
                        catch (Exception ex) { Log?.Invoke("Row write error: " + ex.Message); }
                        cb.PopOldest();
                    }
                }

                DrainEvents();

                if (flushWatch.ElapsedMilliseconds > 1000)
                {
                    _csvWriter.Flush();
                    flushWatch.Restart();
                }
                Thread.Sleep(1);
            }

            // final drain: write whatever is still buffered
            foreach (var cb in _mtwCallbacks)
            {
                PacketItem item;
                while ((item = cb.GetOldest()) != null)
                {
                    try { WriteRow(cb, item); } catch { }
                    cb.PopOldest();
                }
            }
            DrainEvents();
        }

        /// <param name="label">Text label for the marker.</param>
        /// <param name="code">Numeric marker code (Python: 1 = commanded, 2 = actually lit, 4 = pressed).</param>
        /// <param name="pythonUnix">Python's own timestamp (seconds), for checking latency. NaN if unknown.</param>
        public void Mark(string label, int code = 0, double pythonUnix = double.NaN)
        {
            if (!_recording)
            {
                Log?.Invoke("Not recording -- marker ignored.");
                return;
            }
            DateTime t = PreciseClock.UtcNow;
            lock (_eventLock)
            {
                _eventQueue.Enqueue(new EventItem
                {
                    T = t, Code = code, Label = label, Condition = _condition, PythonUnix = pythonUnix
                });
                foreach (var kv in _pendingMarks)
                    kv.Value.Add(new KeyValuePair<int, string>(code, label));
            }
            Log?.Invoke($"Marker {code} '{label}' @ {PreciseClock.ToUnix(t):F3}");
        }

        public void SetCondition(string name)
        {
            _condition = name ?? "";
            if (!_recording) return;
            lock (_eventLock)
            {
                _eventQueue.Enqueue(new EventItem
                {
                    T = PreciseClock.UtcNow, Code = 0, Label = "condition_set",
                    Condition = _condition, PythonUnix = double.NaN
                });
            }
        }

        public void StopRecording()
        {
            if (!_recording) return;
            _stopRequested = true;
            _dataThread?.Join(3000);
            _master.stopRecording();
            // Xsens manual (p.17): after stopRecording the master may "flush" retransmitted data
            // before the log file can be closed. If your .mtb files ever look short, add a wait here.
            _master.closeLogFile();
            _master.gotoConfig();
            _csvWriter?.Close();
            _eventsWriter?.Close();
            _recording = false;
            Log?.Invoke($"Recording stopped. Files:{Environment.NewLine}  {CsvPath}{Environment.NewLine}  {EventsPath}{Environment.NewLine}  {MtbPath}");
        }

        public void Close()
        {
            try
            {
                if (_recording) StopRecording();
                if (_master != null && _master.isRadioEnabled()) _master.disableRadio();
                _control?.close();
            }
            catch (Exception e)
            {
                Log?.Invoke($"Error during shutdown: {e.Message}");
            }
        }
    }

    /// <summary>
    /// Main window: Start / Button Lit / Button Press / Stop.
    /// The Python master script can now do all of this remotely through ImuCommandServer;
    /// the buttons stay for manual testing.
    /// </summary>
    public class RecorderForm : Form
    {
        private readonly XsensRecorder _recorder = new XsensRecorder();
        private ImuCommandServer _server;

        private TextBox _log;
        private Button _startBtn;
        private Button _litBtn;
        private Button _pressBtn;
        private Button _stopBtn;
        private NumericUpDown _sensorCountInput;

        public RecorderForm()
        {
            Text = "Xsens Awinda Recorder";
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

            _startBtn = new Button { Text = "Start", Width = 120, Height = 40 };
            _startBtn.Click += (s, e) => OnStart();
            panel.Controls.Add(_startBtn);

            // Manual test buttons use the same codes Python sends: 2 = button actually lit, 4 = pressed
            _litBtn = new Button { Text = "Button Lit", Width = 120, Height = 40, Enabled = false };
            _litBtn.Click += (s, e) => _recorder.Mark("button_lit", 2);
            panel.Controls.Add(_litBtn);

            _pressBtn = new Button { Text = "Button Press", Width = 120, Height = 40, Enabled = false };
            _pressBtn.Click += (s, e) => _recorder.Mark("button_press", 4);
            panel.Controls.Add(_pressBtn);

            _stopBtn = new Button { Text = "Stop", Width = 120, Height = 40, Enabled = false };
            _stopBtn.Click += (s, e) => OnStop();
            panel.Controls.Add(_stopBtn);

            _recorder.Log = AppendLog;

            // Listen for commands from the Python master script (localhost:5005)
            _server = new ImuCommandServer(_recorder, GetExpectedSensors, AppendLog, SetRunningUi, SetStoppedUi);
            _server.Listen(5005);

            FormClosing += (s, e) => { _server.Stop(); _recorder.Close(); };
        }

        private void AppendLog(string msg)
        {
            if (InvokeRequired) { BeginInvoke(new Action<string>(AppendLog), msg); return; }
            _log.AppendText(msg + Environment.NewLine);
        }

        private int GetExpectedSensors()
        {
            if (InvokeRequired) return (int)Invoke(new Func<int>(GetExpectedSensors));
            return (int)_sensorCountInput.Value;
        }

        private void SetRunningUi()
        {
            if (InvokeRequired) { BeginInvoke(new Action(SetRunningUi)); return; }
            _startBtn.Enabled = false;
            _litBtn.Enabled = true;
            _pressBtn.Enabled = true;
            _stopBtn.Enabled = true;
        }

        private void SetStoppedUi()
        {
            if (InvokeRequired) { BeginInvoke(new Action(SetStoppedUi)); return; }
            _startBtn.Enabled = true;
            _litBtn.Enabled = false;
            _pressBtn.Enabled = false;
            _stopBtn.Enabled = false;
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
                    _recorder.StartRecording();
                    SetRunningUi();
                }
                catch (Exception ex)
                {
                    AppendLog("ERROR: " + ex.Message);
                    BeginInvoke(new Action(() => _startBtn.Enabled = true));
                }
            }) { IsBackground = true };
            t.Start();
        }

        private void OnStop()
        {
            _stopBtn.Enabled = false;
            var t = new Thread(() =>
            {
                _recorder.Close();
                SetStoppedUi();
            }) { IsBackground = true };
            t.Start();
        }
    }
}
