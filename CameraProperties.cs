using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static Headtracker_Console.ExtensionMethods;

namespace Headtracker_Console
{
    public static class CameraProperties
    {
        static string matStr = "C:\\Users\\mufuh\\source\\repos\\Headtracker_Console\\mtxNdist.txt";
        // Initial guess for rotation and translation
        public static Mat rvecInit = Mat.Zeros(3, 1, MatType.CV_64FC1);
        public static Mat tvecInit = Mat.Zeros(3, 1, MatType.CV_64FC1);

        public static double fov = 80.0 * (Math.PI / 180.0);  // convert to radians

        public static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64F);
        public static Mat distCoeffs = Mat.Zeros(1, 5, MatType.CV_64FC1);

        static CameraProperties()
        {
            LoadCalibrationData();
        }

        static int[] depthPoints = { 0, 0, 0, 0 };

        public static Point3f[] objectPoints = new Point3f[] 
        {
            new Point3f(-5f, 0, 0),    // Left
            new Point3f(0, -4f, -3),    // Top
            new Point3f(5f, 0, 0),      // Right
        };

        public static float objWidth = 7.5f;
        public static float objDepth = 4f; // technically its 4 inches, but we cant possible turn our heads that well


        public static Point3f[] SetObjectPointsFromCenter(Point2f[] points)
        {
            // scale distances between each point to one
            Point3f[] newPoints = new Point3f[points.Length];

            for (int i = 0; i < points.Length; i++)
            {
                newPoints[i] = new Point3f(points[i].X - points[0].X, points[i].Y - points[0].Y, depthPoints[i]);

                newPoints[i].X = newPoints[i].X * -1;
                newPoints[i].Y = newPoints[i].Y * 1;
            }

            return newPoints.ToArray();
        }

        public static void LoadCalibrationData()
        {
            Mat newCameraMatrix = new Mat(3, 3, MatType.CV_64F);
            Rect roi = new Rect();

            string[] lines = File.ReadAllLines(matStr);

            // Parse camera matrix (first 3 lines)
            for (int i = 0; i < 3; i++)
            {
                string[] values = lines[i].Trim().Split(' ', (char)StringSplitOptions.RemoveEmptyEntries);
                for (int j = 0; j < 3; j++)
                {
                    cameraMatrix.Set<double>(i, j, double.Parse(values[j]));
                }
            }

            // Parse distortion coefficients (line 4)
            string[] distValues = lines[3].Trim().Split(' ', (char)StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < 5; i++)
            {
                distCoeffs.Set<double>(0, i, double.Parse(distValues[i]));
            }

            // Parse new camera matrix (lines 5-7)
            for (int i = 0; i < 3; i++)
            {
                string[] values = lines[i + 4].Trim().Split(' ', (char)StringSplitOptions.RemoveEmptyEntries);
                for (int j = 0; j < 3; j++)
                {
                    newCameraMatrix.Set<double>(i, j, double.Parse(values[j]));
                }
            }

            // Parse ROI (last line)
            string[] roiValues = lines[7].Trim().Split(' ', (char)StringSplitOptions.RemoveEmptyEntries);
            roi.X = int.Parse(roiValues[0]);
            roi.Y = int.Parse(roiValues[1]);
            roi.Width = int.Parse(roiValues[2]);
            roi.Height = int.Parse(roiValues[3]);
        }
    }
}
