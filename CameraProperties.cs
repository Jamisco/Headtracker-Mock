using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

            ResetPrediction();
        }

        static int[] depthPoints = { 0, 0, 0, 0 };

        public static Point3f[] objectPoints = new Point3f[] 
        {
                new Point3f(5, -3, 0),   // P2 left
                new Point3f(7, 0, 0),       // P1 top
                new Point3f(10, -3, 0)     // P3 right
        };

        public static void SetObjectPointsFromCenter(Point2f[] points)
        {
            return;
            // scale distances between each point to one
            Point3f[] newPoints = new Point3f[points.Length];
            float smallNum = 0;

            for (int i = 0; i < points.Length; i++)
            {
                newPoints[i] = new Point3f(points[i].X - points[0].X, points[i].Y - points[0].Y, depthPoints[i]);

                float num = Math.Max(newPoints[i].X, newPoints[i].Y);

                if (num > smallNum && num > 0)
                {
                    smallNum = num;
                }
            }

            //if (smallNum > 0)
            //{
            //    // divide all points by the smallest number
            //    smallNum = smallNum / 2;

            //    for (int i = 0; i < newPoints.Length; i++)
            //    {
            //        newPoints[i] = newPoints[i].Multiply(1 / smallNum);
            //        newPoints[i].Z = depthPoints[i];
            //    }
            //}

            objectPoints = newPoints.ToArray();
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
        public static void ResetPrediction()
        {

            rvecInit.Set<double>(0, 0, 0.01);
            rvecInit.Set<double>(1, 0, 0.01);
            rvecInit.Set<double>(2, 0, 0.01);

            tvecInit.Set<double>(0, 0, 1.0);  // Larger for translation
            tvecInit.Set<double>(1, 0, 1.0);
            tvecInit.Set<double>(2, 0, 10.0); // Larger Z translation
        }

    }
}
