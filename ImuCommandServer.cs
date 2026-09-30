using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace AwindaMarkerRecorder
{
    /// <summary>
    /// Lets the Python master script drive XsensRecorder over localhost TCP (port 5005).
    ///   START                         -> "OK ..." / "ERR msg"   (connect, wait for sensors, start recording)
    ///   MARK <code> <label> <py_ns>   (no reply)                (same as the old Button Lit / Button Press)
    ///   COND <name>                   (no reply)                (condition name stamped on following rows)
    ///   SAVE                          -> "OK <csv path>"        (stop, finalize files, shut down radio)
    /// Fields are tab-separated, one command per line.
    /// </summary>
    public class ImuCommandServer
    {
        private readonly XsensRecorder _rec;
        private readonly Func<int> _expectedSensors;
        private readonly Action<string> _log;
        private readonly Action _onStarted;
        private readonly Action _onStopped;

        private TcpListener _listener;
        private volatile bool _running;

        public ImuCommandServer(XsensRecorder rec, Func<int> expectedSensors, Action<string> log,
                                Action onStarted, Action onStopped)
        {
            _rec = rec;
            _expectedSensors = expectedSensors;
            _log = log;
            _onStarted = onStarted;
            _onStopped = onStopped;
        }

        public void Listen(int port)
        {
            _listener = new TcpListener(IPAddress.Loopback, port);
            _listener.Start();
            _running = true;
            _log?.Invoke($"Command server listening on 127.0.0.1:{port} (waiting for Python).");
            Task.Run(async () =>
            {
                while (_running)
                {
                    try
                    {
                        TcpClient c = await _listener.AcceptTcpClientAsync();
                        _ = Task.Run(() => Serve(c));
                    }
                    catch { if (!_running) break; }
                }
            });
        }

        public void Stop()
        {
            _running = false;
            try { _listener.Stop(); } catch { }
        }

        private void Serve(TcpClient client)
        {
            client.NoDelay = true;
            _log?.Invoke("Python connected.");
            using (client)
            using (var rd = new StreamReader(client.GetStream(), Encoding.UTF8))
            using (var wr = new StreamWriter(client.GetStream(), new UTF8Encoding(false)) { AutoFlush = true })
            {
                string line;
                while ((line = rd.ReadLine()) != null)
                {
                    string[] p = line.Split('\t');
                    try
                    {
                        switch (p[0])
                        {
                            case "START":
                                {
                                    int expected = _expectedSensors();
                                    _rec.Connect();
                                    int n = _rec.WaitForSensors(expected, 30);
                                    _rec.StartRecording();
                                    _onStarted?.Invoke();
                                    wr.WriteLine(n >= expected
                                        ? "OK"
                                        : $"OK but only {n} of {expected} sensors connected");
                                    break;
                                }
                            case "MARK":
                                {
                                    int code = int.Parse(p[1], CultureInfo.InvariantCulture);
                                    string label = p.Length > 2 ? p[2] : "";
                                    double pyUnix = double.NaN;
                                    long ns;
                                    if (p.Length > 3 && long.TryParse(p[3], out ns)) pyUnix = ns / 1e9;
                                    _rec.Mark(label, code, pyUnix);
                                    break;
                                }
                            case "COND":
                                _rec.SetCondition(p.Length > 1 ? p[1] : "");
                                break;
                            case "SAVE":
                                _rec.Close();
                                _onStopped?.Invoke();
                                wr.WriteLine("OK " + _rec.CsvPath);
                                break;
                        }
                    }
                    catch (Exception ex)
                    {
                        _log?.Invoke($"Command '{p[0]}' failed: {ex.Message}");
                        if (p[0] == "START" || p[0] == "SAVE") wr.WriteLine("ERR " + ex.Message);
                    }
                }
            }
            _log?.Invoke("Python disconnected.");
        }
    }
}
