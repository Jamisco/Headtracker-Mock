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

            Console.SetWindowSize(50, 50);
            // do some work

            vidStream = new HeadTracker(1);
            vidStream.StartTracking();

            // Usage example

        }

        private static void CurrentDomain_ProcessExit(object sender, EventArgs e)
        {
            vidStream.ReleaseResources();
        }

        public static Mat MapPointToMatrix(Point2f d, Point2f target, Mat m)
        {
            if (m.Rows != 1 || m.Cols != 3)
            {
                throw new ArgumentException("Matrix m must be 1x3.");
            }

            // Define scaling factors based on target point
            float s_x = 1.0f / target.X;
            float s_y = 1.0f / target.Y;

            // Normalize input point d with respect to target
            float normalizedX = s_x * d.X;
            float normalizedY = s_y * d.Y;

            // Ensure values are capped at 1
            //normalizedX = Math.Min(normalizedX, 1.0f);
            //normalizedY = Math.Min(normalizedY, 1.0f);

            // Get values from m
            float m1 = m.At<float>(0, 0);
            float m2 = m.At<float>(0, 1);
            float m3 = m.At<float>(0, 2);

            // Compute weighted values for each element of m based on normalized coordinates
            float result1 = normalizedX * m1;
            float result2 = normalizedY * m2;
            float result3 = (normalizedX + normalizedY) / 2.0f * m3; // Averaging effect

            // Create result matrix
            Mat result = new Mat(1, 3, MatType.CV_32F);
            result.Set(0, 0, result1);
            result.Set(0, 1, result2);
            result.Set(0, 2, result3);

            return result;
        }
    }

    

}
