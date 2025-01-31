using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static HeadTracker;

namespace Headtracker_Console
{
    public static class HeadPose
    {
        static double fov = 60.0; // degrees
        static int imageWidth = 1920;
        static int imageHeight = 1080;

        // Calculate focal length in pixels
        static double focalLength = imageWidth / (2 * Math.Tan(fov * Math.PI / 360.0));

        // Create camera matrix
        static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
        static Mat distCoeffs = Mat.Zeros(4, 1, MatType.CV_64FC1);

        static Mat objectPoints = new Mat(3, 3, MatType.CV_32FC1);

        // Create 3D model points (assuming equilateral triangle in 3D space)
        static float[] points = new float[]
        {
                0, -9, -4,     // Top center point (x,y,z)
                6, 0, 0,      // Bottom right point (x,y,z)
                -6, 0, 0      // Bottom left point (x,y,z)
        };

        static HeadPose()
        {
            cameraMatrix.SetArray(new double[]
                                  {
                                        focalLength, 0, imageWidth / 2.0,
                                        0, focalLength, imageHeight / 2.0,
                                        0, 0, 1
                                   });

            objectPoints.SetArray(points);

        }
        // Initialize rotation and translation vectors to zeros
        static Mat rvec = Mat.Zeros(3, 1, MatType.CV_64FC1);
        static Mat tvec = Mat.Zeros(3, 1, MatType.CV_64FC1);
        public static void GetHeadPose(Triangle cur, out Point3d r, out Point3d t)
        {
            Cv2.SolvePnP(objectPoints, cur.PointsMatrix, cameraMatrix, distCoeffs, rvec,     tvec, useExtrinsicGuess: true);

            Mat rotationMatrix = new Mat();
            // Convert rotation vector to rotation matrix
            Cv2.Rodrigues(rvec, rotationMatrix);

            // Extract Euler angles (Pitch, Yaw, Roll)
            Point3d eulerAngles = RotationMatrixToEulerAngles(rotationMatrix);

            // Translation vector
            Point3d translation = new Point3d(
                tvec.Get<double>(0, 0),
                tvec.Get<double>(1, 0),
                tvec.Get<double>(2, 0)
            );

            r = eulerAngles - rOffset;
            t = translation - tOffset;

            // Use eulerAngles and translation as needed
        }

        static Point3d tOffset = new Point3d();
        static Point3d rOffset = new Point3d();

        public static void SetOffset(Triangle center)
        {
            GetHeadPose(center, out Point3d r, out Point3d t);
            tOffset = t;
            rOffset = r;
        }

        public static void ClearPredictions()
        {
            rvec = Mat.Zeros(3, 1, MatType.CV_64FC1);
            tvec = Mat.Zeros(3, 1, MatType.CV_64FC1);

            tOffset = new Point3d();
            rOffset = new Point3d();
        }

        private static Point3d RotationMatrixToEulerAngles(Mat R)
        {
            double sy = Math.Sqrt(R.Get<double>(0, 0) * R.Get<double>(0, 0) + R.Get<double>(1, 0) * R.Get<double>(1, 0));
            bool singular = sy < 1e-6;

            double x, y, z;
            if (!singular)
            {
                x = Math.Atan2(R.Get<double>(2, 1), R.Get<double>(2, 2));
                y = Math.Atan2(-R.Get<double>(2, 0), sy);
                z = Math.Atan2(R.Get<double>(1, 0), R.Get<double>(0, 0));
            }
            else
            {
                x = Math.Atan2(-R.Get<double>(1, 2), R.Get<double>(1, 1));
                y = Math.Atan2(-R.Get<double>(2, 0), sy);
                z = 0;
            }

            // Convert radians to degrees
            return new Point3d(
                x * (180.0 / Math.PI),
                y * (180.0 / Math.PI),
                z * (180.0 / Math.PI)
            );
        }
    }
}
