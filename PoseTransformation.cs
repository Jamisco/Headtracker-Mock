using OpenCvSharp;
using Emgu;
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
        }

        #region Solve PnP related
        static string matStr = "C:\\Users\\mufuh\\source\\repos\\Headtracker_Console\\mtxNdist.txt";
        // Initial guess for rotation and translation
        static Mat rvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);
        static Mat tvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);

        static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64F);
        static Mat distCoeffs = Mat.Zeros(1, 5, MatType.CV_64FC1);

        public static Point3d tOffset = new Point3d();
        public static Point3d rOffset = new Point3d();

        private static Point2f[] centerPoints;

        static float depth = -100;
        public static void ResetPrediction()
        {

            rvecGuess.Set<double>(0, 0, 0);
            rvecGuess.Set<double>(1, 0, 0);
            rvecGuess.Set<double>(2, 0, -8);

            tvecGuess.Set<double>(0, 0, 0.1f);
            tvecGuess.Set<double>(1, 0, 0.1f);
            tvecGuess.Set<double>(2, 0, 10f);

            //depth += .1f;
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

        static float tx = 2;
        static float ty = -5;

        public static void SetPrediction(Point2f[] points)
        {
            Point2f centriod = GetCentriod(points);

            Point2f frameDiff = FRAMECENTER - centriod;

            float num = 5;
            frameDiff = frameDiff.Multiply(1 / num);

            tvecGuess.Set<double>(0, 0, -2);
            tvecGuess.Set<double>(1, 0, 5);
            tvecGuess.Set<double>(2, 0, -40);

            rvecGuess.Set<double>(0, 0, 0);
            rvecGuess.Set<double>(1, 0, 0);
            rvecGuess.Set<double>(2, 0, 0);

        }



        public static void EstimateTransformation(
                    Point2f[] curPoints,   // Current 2D points
                    out Point3d r,               // Rotation angles
                    out Point3d t)               // Translation vector
        {
            //ResetPrediction();
            Point3f[] objPoints = CameraProperties.objectPoints.ToArray();
            //Point3f[] objPoints = CameraProperties.SetObjectPointsFromCenter(curPoints);

            // used to scale the object values if need me
            float num = 1f / 1f;
            for (int i = 0; i < objPoints.Length; i++)
            {
                Point3f p = objPoints[i];

                objPoints[i] = p.Multiply(1f / num);
            }

            // predict only the first frame
            if (frameAfterCenter == 0)
            {
                SetPrediction(curPoints);
            }

            string rd = rvecGuess.Dump();
            string td = tvecGuess.Dump();

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
                    useExtrinsicGuess: true,  // useExtrinsicGuess = true
                    flags: SolvePnPFlags.Iterative
                );

                // Get the results  
                Mat rvec = rvecOutput.GetMat();
                Mat tvec = tvecOutput.GetMat();

                string rd2 = rvec.Dump();
                string td2 = tvec.Dump();

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

                ProjectPoints(rvec, tvec);

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

                if (frameAfterCenter == 0)
                {
                    rOffset = r;
                    tOffset = t;
                }

                //r = r - rOffset;
                //t = t - tOffset;

                frameAfterCenter++;
            }
        }
        private static void ProjectPoints(Mat rvec, Mat tvec)
        {
            // Create correct size output array (same number of points as object points)
            Point2f[] projectedPoints = new Point2f[CameraProperties.objectPoints.Length];

            InputArray objectPointsInput = InputArray.Create(CameraProperties.objectPoints);

            // Create output array as Mat instead of Point2f[]
            Mat projectedPointsMat = new Mat(CameraProperties.objectPoints.Length, 1, MatType.CV_64FC2);
            OutputArray projectedPointsOutput = OutputArray.Create(projectedPointsMat);

            string rd = rvec.Dump();
            string td = tvec.Dump();

            Cv2.ProjectPoints(
                objectPointsInput,
                rvec,
                tvec,
                cameraMatrix,
                distCoeffs,
                projectedPointsOutput
            );


            // Convert Mat to Point2f array
            Mat resultMat = projectedPointsOutput.GetMat();
            string md = resultMat.Dump();

            // For CV_32FC2, each row contains both x and y
            for (int i = 0; i < resultMat.Rows; i++)
            {
                Vec2f point = resultMat.Get<Vec2f>(i, 0);
                projectedPoints[i] = new Point2f(point[0], point[1]);
            }

            Mat frame = HeadTracker.CloneFrame.EmptyClone();

            if(HeadTracker.Shape2Use == ShapeType.TShape)
            {
                TShape shape = new TShape(projectedPoints.ToList());
                shape.DrawShape(frame, Scalar.Green, true);

            }
            else
            {
                Polygon shape = new Polygon(projectedPoints.ToList());
                shape.DrawShape(frame, Scalar.Green, true);

            }

            Cv2.ImShow("Projected Points", frame);
            Cv2.WaitKey(1);

        }

        public static void EstimateTransformation2(Point2f[] curPoints, out Point3d r, out Point3d t)
        {
            Point2f centriod = GetCentriod(curPoints);

            float angle = 100;

            float yaw = (curPoints[1].X - centriod.X);
            float pitch = 0;

            Point2f P0 = curPoints[0];    // Left base
            Point2f P1 = curPoints[1];    // Top point
            Point2f P2 = curPoints[2];    // Right base
            // Vector math with Point2f
            Point2f baseVector = P2 - P0;
            Point2f topVector = P1 - P0;

            // Projection formula
            float dot = (topVector.X * baseVector.X + topVector.Y * baseVector.Y);
            float baseLengthSq = (baseVector.X * baseVector.X + baseVector.Y * baseVector.Y);
            float t1 = dot / baseLengthSq;

            // Intersection point
            Point2f intersection = new Point2f(
                P0.X + baseVector.X * t1,
                P0.Y + baseVector.Y * t1
            );

            float height = (float)Point2f.Distance(P1, intersection);
            float width = (float)Point2f.Distance(P0, P2);

            pitch = ((height / width) - .75f) * angle;

            float rAngle = .2f;
            float roll = (curPoints[0] - curPoints[2]).Y * rAngle;

            // for yaw figure out left/right deviation from center center
            // for pitch figure out up//down deviation from center perdicular or center of shape
            // Convert to degrees
            r = new Point3d(
                pitch,
                -yaw ,
                roll
            );

            // Translation
            t = new Point3d(
                0,0,0
            );

        }

        public static void EstimateTransformation2(TShape center, TShape cur, out Point3d r, out Point3d t)
        {
            float pitch, yaw, roll;

            Point2f centerDiff = (cur.Centroid - cur.TopCentroid) - (center.Centroid - center.TopCentroid);

            float angle = 2f;
            float ra = .3f;

            pitch = centerDiff.Y * angle;
            yaw = centerDiff.X * angle;

            roll = ((cur.Points[0] - cur.Points[2]).Y - (center.Points[0] - center.Points[2]).Y) * ra;

            // for yaw figure out left/right deviation from center center
            // for pitch figure out up//down deviation from center perdicular or center of shape
            // we should be to introduce counter weights for each transformations,
            // for example for every degree in roll reduce yaw by 2 degrees, or a percentage etc.
            // this way we reduce the problem of drift.
            // Now this might require some calibration, but we can offset this.

            // we can also reproject back our estimated transformation back into the current shape, then use that to recalculate translation
            r = new Point3d(
                pitch,
                yaw,
                roll
            );

            // Translation
            t = new Point3d(
                0, 0, 0
            );

        }

        #endregion

        static int frameAfterCenter = 0;

        public static void SetCenter(Point2f[] points)
        {
            centerPoints = points;
            ResetPrediction();
        }

        static float z = 50;



        public static void ClearOffsets()
        {
            frameAfterCenter = 0;
        }
    }
}
