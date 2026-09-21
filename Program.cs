using System;
using System.Windows.Forms;

namespace AwindaMarkerRecorder
{
    static class Program
    {
        [STAThread]
        static void Main()
        {
            // Surface ANY startup failure in a visible dialog instead of the
            // process silently exiting with nothing shown.
            AppDomain.CurrentDomain.UnhandledException += (s, e) =>
            {
                MessageBox.Show("Unhandled exception:\n\n" + e.ExceptionObject,
                    "AwindaMarkerRecorder - Fatal Error");
            };

            try
            {
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
                Application.ThreadException += (s, e) =>
                {
                    MessageBox.Show("UI thread exception:\n\n" + e.Exception,
                        "AwindaMarkerRecorder - Fatal Error");
                };
                Application.Run(new RecorderForm());
            }
            catch (Exception ex)
            {
                MessageBox.Show("Startup failed:\n\n" + ex,
                    "AwindaMarkerRecorder - Fatal Error");
            }
        }
    }
}