using OpenCvSharp;
using System;

namespace Headtracker_Console
{
    internal class Program
    {
        static HeadTracker vidStream;
        static void Main(string[] args)
        {
            AppDomain.CurrentDomain.ProcessExit
                += new EventHandler(CurrentDomain_ProcessExit);

            Console.SetWindowSize(50, 20);
            // do some work

            vidStream = new HeadTracker(0);
            vidStream.StartTracking();

            // Usage example

        }

        private static void CurrentDomain_ProcessExit(object sender, EventArgs e)
        {
            vidStream.ReleaseResources();
        }
    }

}
