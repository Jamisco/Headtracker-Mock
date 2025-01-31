using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using Headtracker_Console;

using static HeadTracker;

namespace Headtracker_Console
{
    public class PoseTransformation
    {
        static string matStr = "C:\\Users\\mufuh\\source\\repos\\Headtracker_Console\\mtxNdist.txt";
        // Initial guess for rotation and translation
        static Mat rvecInit = Mat.Zeros(3, 1, MatType.CV_64FC1);
        static Mat tvecInit = Mat.Zeros(3, 1, MatType.CV_64FC1);

        static double fov = 80.0 * (Math.PI / 180.0);  // convert to radians

        public static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64F);
        public static Mat distCoeffs = Mat.Zeros(1, 5, MatType.CV_64FC1);

        static PoseTransformation()
        {
            LoadCalibrationData();

            ResetPrediction();
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

        public static void GetCamCal()
        {
            // read file 
        }



        // Image dimensions
        static int imageWidth = FRAMEWIDTH;  // Set your image width
        static int imageHeight = FRAMEHEIGHT; // Set your image height

        static int dept = 1;

        public static double focalLen = 90.308;
        static double fl = 0;
        static float scale = 1.0f;  // increment this: 1, 2, 5, 10, 20, etc.
        static float inc = 600;
        public static void EstimateTransformation(
                    Point2f[] currentTriangle,   // Current 2D points
                    out Point3d r,               // Rotation angles
                    out Point3d t)               // Translation vector
        {

            // Create 3D model points (assuming equilateral triangle in 3D space)
            // Object Points - isosceles triangle with depth
            Point3f[] objectPoints;

            if(TEST)
            {
                objectPoints = CreateScaledObjectPoints();
            }
            else
            {
                if(hasCenter)
                {
                    objectPoints = CreateScaledObjectPoints(cp);
                    SetPredictionValues(currentTriangle, rvecInit, tvecInit);

                }
                else
                {
                    objectPoints = new Point3f[]
{
                    new Point3f(0, -0.09f, -0.04f),    // Top
                    new Point3f(0.06f, 0, 0),          // Right
                    new Point3f(-0.06f, 0, 0)          // Left
};
                }

            }

            // Convert arrays to InputArray
            using (InputArray objectPointsInput = InputArray.Create(objectPoints))
            using (InputArray imagePointsInput = InputArray.Create(currentTriangle))
            using (InputArray cameraMatrixInput = InputArray.Create(cameraMatrix))
            using (InputArray distCoeffsInput = InputArray.Create(distCoeffs))
            using (OutputArray rvecOutput = OutputArray.Create(rvecInit))
            using (OutputArray tvecOutput = OutputArray.Create(tvecInit))
            {
                // Solve for current pose with initial guess
                Cv2.SolvePnP(
                    objectPointsInput,
                    imagePointsInput,
                    cameraMatrixInput,
                    distCoeffsInput,
                    rvecOutput,
                    tvecOutput,
                    true,  // useExtrinsicGuess = true
                    SolvePnPFlags.Iterative
                );

                // Get the results
                Mat rvec = rvecOutput.GetMat();
                Mat tvec = tvecOutput.GetMat();

                // Convert rotation vector to matrix
                Mat rotationMatrix = new Mat();
                Cv2.Rodrigues(rvec, rotationMatrix);

                string camDump = cameraMatrix.Dump();
                string distDump = distCoeffs.Dump();
                string rotDump = rotationMatrix.Dump();

                // Extract Euler angles
                // For pitch (X rotation)
                double pitch = Math.Atan2(rotationMatrix.Get<double>(2, 1),
                                         rotationMatrix.Get<double>(2, 2));

                // For yaw (Y rotation)
                double yaw = Math.Atan2(-rotationMatrix.Get<double>(2, 0),
                                        Math.Sqrt(rotationMatrix.Get<double>(2, 1) * rotationMatrix.Get<double>(2, 1) +
                                                 rotationMatrix.Get<double>(2, 2) * rotationMatrix.Get<double>(2, 2)));

                // For roll (Z rotation)
                double roll = Math.Atan2(rotationMatrix.Get<double>(1, 0),
                                        rotationMatrix.Get<double>(0, 0));

                // For pitch
                if (pitch >= 0)
                    pitch = Math.Min(pitch, pitchLimit);
                else
                    pitch = Math.Max(pitch, -pitchLimit);
                     pitch = pitch / 2;

                // For yaw
                if (yaw >= 0)
                    yaw = Math.Min(yaw, yawLimit);
                else
                    yaw = Math.Max(yaw, -yawLimit);

                // Convert to degrees
                r = new Point3d(
                    pitch * 180.0 / Math.PI,
                    yaw * 180.0 / Math.PI,
                    roll * 180.0 / Math.PI
                );

                // Translation
                t = new Point3d(
                    tvec.Get<double>(0, 0),
                    tvec.Get<double>(1, 0),
                    tvec.Get<double>(2, 0)
                );

                r = r - rOffset;
                t = t - tOffset;
            }
        }

        static Point3d tOffset = new Point3d();
        static Point3d rOffset = new Point3d();

        static bool hasCenter = false;
        static Point2f[] cp;
        public static void SetOffset(Triangle center)
        {
            ClearPredictions();
            cp = center.Points;
            centerX = center.Centroid.X;
            centerY = center.Centroid.Y;

            hasCenter = true;
            EstimateTransformation(center.Points, out Point3d r, out Point3d t);
            tOffset = t;
            rOffset = r;

        }

        public static void ClearPredictions()
        {
            tOffset = new Point3d();
            rOffset = new Point3d();
            hasCenter = false;

            maxRightX = baseMax;
            maxLeftX = baseMax;
            maxLeftX = baseMax;
            maxDownY = baseMax;

            ResetPrediction();
        }

        private static float baseMax = 20;
        private static float centerX;
        private static float centerY;
        private static float maxRightX = baseMax;
        private static float maxLeftX = baseMax;
        private static float maxUpY = baseMax;
        private static float maxDownY = baseMax;
        private static float yawLimit;
        private static float pitchLimit;

        private static readonly float maxYawAngle = 60f;   // maximum yaw in degrees
        private static readonly float maxPitchAngle = 60f; // maximum pitch in degrees

        static void GetCenter(Point2f[] currentTriangle, out float cx, out float cy)
        {
            cx = (currentTriangle[0].X + currentTriangle[1].X + currentTriangle[2].X) / 3;
            cy = (currentTriangle[0].Y + currentTriangle[1].Y + currentTriangle[2].Y) / 3;
        }

        public static void SetPredictionValues(Point2f[] currentTriangle, Mat rvecInit, Mat tvecInit)
        {
            GetCenter(currentTriangle, out float currentX, out float currentY);
            int negx = 1;
            int negy = 1;

            // Yaw calculation (positive for right, negative for left)
            float xDisplacement = currentX - centerX;

            if(xDisplacement > 0)
            {
                xDisplacement = Math.Abs(xDisplacement);
                maxRightX = Math.Abs((xDisplacement > maxRightX) ? xDisplacement : maxRightX);
            }else if(xDisplacement < 0)
            {
                xDisplacement = Math.Abs(xDisplacement);
                negx = -1;
                maxLeftX = Math.Abs((xDisplacement > maxLeftX) ? xDisplacement : maxLeftX);
            }

            float xp = (negx >= 0) ? xDisplacement / maxRightX : xDisplacement / maxLeftX;
            yawLimit = (float)(maxYawAngle * Math.Abs(xp) * (Math.PI / 180.0));  // Convert to radians
            yawLimit *= negy;

            // Pitch calculation (negative for up, positive for down)
            float yDisplacement = currentY - centerY;

            if(yDisplacement > 0)
            {
                yDisplacement = Math.Abs(yDisplacement);
                maxUpY = Math.Abs((yDisplacement > maxUpY) ? yDisplacement : maxUpY);
            }
            else if (yDisplacement < 0)
            {
                yDisplacement = Math.Abs(yDisplacement);
                negy = -1;
                maxDownY = Math.Abs((yDisplacement > maxDownY) ? yDisplacement: maxDownY);

            }

            float yRange = (negy >= 0) ? yDisplacement / maxUpY: yDisplacement / maxDownY;
            pitchLimit = (float)(maxPitchAngle * Math.Abs(yRange) * (Math.PI / 180.0));  // Convert to radians
            pitchLimit *= negy;

            //pitchLimit = Math.Max(pitchLimit, 0.1f);
            //yawLimit = Math.Max(yawLimit, 0.1f);

            // Set prediction values (now in radians)
            rvecInit.Set<double>(0, 0, pitchLimit * negy);   // pitch (negative for up)
            rvecInit.Set<double>(1, 0, yawLimit / 2 * negx);     // yaw (negative for left)
            rvecInit.Set<double>(1, 0, (yawLimit / 2 * negx) / 2);     // yaw (negative for left)
        }


        private static Point2f CalculateCentroid(Point2f[] points)
        {
            float sumX = 0, sumY = 0;
            foreach (var point in points)
            {
                sumX += point.X;
                sumY += point.Y;
            }
            return new Point2f(sumX / points.Length, sumY / points.Length);
        }

        private static Mat CalculateCovarianceMatrix(Point2f[] points1, Point2f[] points2)
        {
            Mat covariance = Mat.Zeros(2, 2, MatType.CV_32FC1);

            for (int i = 0; i < points1.Length; i++)
            {
                Mat p1 = new Mat(2, 1, MatType.CV_32F);
                Mat p2 = new Mat(2, 1, MatType.CV_32F);

                p1.Set(0, 0, points1[i].X);
                p1.Set(1, 0, points1[i].Y);
                p2.Set(0, 0, points2[i].X);
                p2.Set(1, 0, points2[i].Y);

                covariance += p2 * p1.T();
            }

            return covariance;
        }

        private static Point2f TransformPoint(Point2f point, Mat rotation)
        {
            Mat p = new Mat(2, 1, MatType.CV_32F);
            p.Set(0, 0, point.X);
            p.Set(1, 0, point.Y);

            Mat transformed = rotation * p;
            return new Point2f(
                (float)transformed.Get<float>(0, 0),
                (float)transformed.Get<float>(1, 0));
        }
    }
}
