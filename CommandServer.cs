using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace AwindaMarkerRecorder
{
    /// <summary>
    /// Line-based TCP command server on 127.0.0.1 so the Python master script can
    /// drive the Awinda recorder (connect / arm / start / mark / stop).
    ///
    /// Protocol: one ASCII command per line, one ASCII reply line per command.
    ///   PING                     -> OK PONG
    ///   SETDIR <folder>          -> OK DIR <folder>
    ///   CONNECT <n_sensors>      -> OK CONNECTED <n>       (slow: radio + sensor join)
    ///   PREPARE <session_name>   -> OK PREPARED            (slow: config + file create)
    ///   GO                       -> OK GO <unix_seconds>   (fast: actually starts recording)
    ///   MARK <code> <label>      -> OK MARK <unix_seconds>
    ///   STOP                     -> OK STOPPED
    ///   QUIT                     -> OK BYE
    /// Any failure replies "ERR <message>".
    /// </summary>
    public class CommandServer
    {
        private readonly XsensRecorder _rec;
        private readonly int _port;
        private TcpListener _listener;
        private Thread _acceptThread;
        private volatile bool _stopping;

        public Action<string> Log;

        public CommandServer(XsensRecorder recorder, int port = 5555)
        {
            _rec = recorder;
            _port = port;
        }

        public void Start()
        {
            _listener = new TcpListener(IPAddress.Loopback, _port);
            _listener.Start();
            _acceptThread = new Thread(AcceptLoop) { IsBackground = true };
            _acceptThread.Start();
            Log?.Invoke($"Command server listening on 127.0.0.1:{_port}");
        }

        public void Stop()
        {
            _stopping = true;
            try { _listener?.Stop(); } catch { }
        }

        private void AcceptLoop()
        {
            while (!_stopping)
            {
                TcpClient client;
                try { client = _listener.AcceptTcpClient(); }
                catch { break; }

                client.NoDelay = true;   // no Nagle delay -- markers must be prompt
                var t = new Thread(() => HandleClient(client)) { IsBackground = true };
                t.Start();
            }
        }

        private void HandleClient(TcpClient client)
        {
            Log?.Invoke("Python client connected.");
            try
            {
                using (client)
                using (var ns = client.GetStream())
                using (var reader = new StreamReader(ns, Encoding.ASCII))
                using (var writer = new StreamWriter(ns, Encoding.ASCII) { AutoFlush = true })
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        string reply;
                        try { reply = Dispatch(line.Trim()); }
                        catch (Exception ex)
                        {
                            reply = "ERR " + ex.Message.Replace("\r", " ").Replace("\n", " ");
                            Log?.Invoke("ERROR: " + ex.Message);
                        }
                        writer.WriteLine(reply);
                    }
                }
            }
            catch (Exception ex)
            {
                Log?.Invoke("Client connection ended: " + ex.Message);
            }
            Log?.Invoke("Python client disconnected.");
        }

        private string Dispatch(string line)
        {
            if (line.Length == 0) return "OK";

            int sp = line.IndexOf(' ');
            string cmd = (sp < 0 ? line : line.Substring(0, sp)).ToUpperInvariant();
            string arg = sp < 0 ? "" : line.Substring(sp + 1).Trim();

            switch (cmd)
            {
                case "PING":
                    return "OK PONG";

                case "SETDIR":
                    _rec.LogDir = arg;
                    Directory.CreateDirectory(arg);
                    return "OK DIR " + arg;

                case "CONNECT":
                {
                    int n;
                    if (!int.TryParse(arg, out n) || n < 1) n = 1;
                    _rec.Connect();
                    int found = _rec.WaitForSensors(n, 60);
                    if (found < n)
                        return $"ERR only {found} of {n} sensor(s) connected";
                    return $"OK CONNECTED {found}";
                }

                case "PREPARE":
                    _rec.Prepare(arg);
                    return "OK PREPARED";

                case "GO":
                    return $"OK GO {_rec.Go():F6}";

                case "MARK":
                {
                    int sp2 = arg.IndexOf(' ');
                    string codeStr = sp2 < 0 ? arg : arg.Substring(0, sp2);
                    string label   = sp2 < 0 ? ""  : arg.Substring(sp2 + 1);
                    int code;
                    if (!int.TryParse(codeStr, out code)) { code = 0; label = arg; }
                    double t = _rec.Mark(code, label);
                    return $"OK MARK {t:F6}";
                }

                case "STOP":
                    _rec.StopRecording();
                    return "OK STOPPED";

                case "QUIT":
                    _rec.Close();
                    return "OK BYE";

                default:
                    return "ERR unknown command: " + cmd;
            }
        }
    }
}
