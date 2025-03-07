using OpenCvSharp;
using System;
using System.IO;
using System.Linq;
using static Headtracker_Console.ExtensionMethods;
using static Headtracker_Console.HeadTracker;

namespace Headtracker_Console
{
    public static class PoseTransformation
    {
        static PoseTransformation()
        {
            LoadCalibrationData();
            ResetPrediction();
        }

        #region Solve PnP related
        static string matStr = "C:\\Users\\mufuh\\source\\repos\\Headtracker_Console\\mtxNdist.txt";
        // Initial guess for rotation and translation
        static Mat rvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);
        static Mat tvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);

        static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64F);
        static Mat distCoeffs = Mat.Zeros(1, 5, MatType.CV_64FC1);

        static Point3d tOffset = new Point3d();
        static Point3d rOffset = new Point3d();

        private static Point2f[] centerPoints;

        public static void ResetPrediction()
        {
            rvecGuess.Set<double>(0, 0, 0);
            rvecGuess.Set<double>(1, 0, 0);
            rvecGuess.Set<double>(2, 0, 0);

            tvecGuess.Set<double>(0, 0, 1.0);  // Larger for translation
            tvecGuess.Set<double>(1, 0, 1.0);
            tvecGuess.Set<double>(2, 0, 1.0); // Larger Z translation
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

        static float scale = 1.0f;  // increment this: 1, 2, 5, 10, 20, etc.
        public static void EstimateTransformation(
                    Point2f[] curPoints,   // Current 2D points
                    out Point3d r,               // Rotation angles
                    out Point3d t)               // Translation vector
        {
           // CameraProperties.SetObjectPointsFromCenter(centerPoints);
            Point3f[] objPoints = CameraProperties.objectPoints.ToArray();

            int num = 10;
            for (int i = 0; i < objPoints.Length; i++)
            {
                Point3f p = objPoints[i];

                objPoints[i] = p.Multiply(1f / num);
            }

            SetPrediction(curPoints);

            // Convert arrays to InputArray
            using (InputArray objectPointsInput = InputArray.Create(objPoints))
            using (InputArray imagePointsInput = InputArray.Create(curPoints))
            using (InputArray cameraMatrixInput = InputArray.Create(cameraMatrix))
            using (InputArray distCoeffsInput = InputArray.Create(distCoeffs))
            using (OutputArray rvecOutput = OutputArray.Create(rvecGuess))
            using (OutputArray tvecOutput = OutputArray.Create(tvecGuess))
            {
                // Solve for current pose with initial guess
                Cv2.SolvePnP(
                    objectPointsInput,
                    imagePointsInput,
                    cameraMatrixInput,
                    distCoeffsInput,
                    rvecOutput,
                    tvecOutput,
                    false,  // useExtrinsicGuess = true
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

                //rvecInit = rvec;
                //tvecInit = tvec;
            }
        }

        #endregion

        public static void EstimateTransformation2(
                    Point2f[] curPoints,   // Current 2D points
                    out Point3d r,               // Rotation angles
                    out Point3d t)
        {
            r = new Point3d();
            t = new Point3d();

            // just hard code it back

            // yaw is y axis of base vector
            // use iterative method to find the best yaw /pitch
            // we know which axis will be more depending on centriod diff
        }

        public static void SetCenter(Point2f[] points)
        {
            centerPoints = points;
        }

        public static void SetPrediction(Point2f[] points)
        {
            TShape shape = new TShape(points.ToList());

            Point2f mid = MidPoint2f(points[0], points[2]);

            Point2f baseVector = points[2] - points[0];
            Point2f heightVector = points[1] - mid;

            Point2f cmid = MidPoint2f(centerPoints[0], centerPoints[2]);

            Point2f cbaseVector = centerPoints[2] - centerPoints[0];
            Point2f cheightVector = centerPoints[1] - cmid;

            float roll = baseVector.Y;
            float pitch = ((heightVector.Y - cheightVector.Y) / cheightVector.Y) * 60;
            float yaw = ((baseVector.X - cbaseVector.X) / cbaseVector.X) * 60;


            if(pitch > 90)
            {
                pitch = ((cheightVector.Y - heightVector.Y) / cheightVector.Y) * 60;
            }

            if(yaw > 90)
            {
                yaw = ((cbaseVector.X - baseVector.X) / cbaseVector.X) * 60;
            }

            rvecGuess.Set<double>(0, 0, pitch);
            rvecGuess.Set<double>(1, 0, yaw);
            rvecGuess.Set<double>(2, 0, roll);

            tvecGuess = new Mat(3, 1, MatType.CV_64FC1);

            tvecGuess.Set<double>(0, 0, 1.0);  // Larger for translation
            tvecGuess.Set<double>(1, 0, 1.0);
            tvecGuess.Set<double>(2, 0, 1.0); // Larger Z translation
        }

        public static void ClearOffsets()
        {
            rOffset = new Point3d();
            tOffset = new Point3d();
            ResetPrediction();
        }
    }
}
