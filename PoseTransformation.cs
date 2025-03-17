using OpenCvSharp;
using Emgu;
using System;
using System.IO;
using System.Linq;
using static Headtracker_Console.ExtensionMethods;
using static Headtracker_Console.HeadTracker;
using Emgu.CV.CvEnum;
using static Headtracker_Console.Calibrator;

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

        public static Point3f tOffset = new Point3f();
        public static Point3f rOffset = new Point3f();

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
                out Point3f r,               // Rotation angles
                out Point3f t)               // Translation vector
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
                r = new Point3f(
                    (float)(pitch * 180.0 / Math.PI),
                    (float)(yaw * 180.0 / Math.PI),
                    (float)(roll * 180.0 / Math.PI)
                );

                // Translation
                t = new Point3f(
                    (float)tvec.Get<double>(0, 0),
                    (float)tvec.Get<double>(1, 0),
                    (float)tvec.Get<double>(2, 0)
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

            if (HeadTracker.Shape2Use == ShapeType.TShape)
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

        public static void GetPureRotation(TShape center, TShape cur, out Point3f r)
        {
            float pitch, yaw, roll;

            // the yaw angle can be derived by using the depth and get the max distance with the center can deviate from the center line. Said max distance is the max distance of the depth, or the depth from the base points to the back point

            Point2f cp1 = center.HeightVector;
            Point2f cp2 = cur.HeightVector;

            // modify matrix to account for 00 top left system

            Point2f centerDiff = cur.CenterVector - center.CenterVector;
            float angle = 2f;
            pitch = centerDiff.Y * angle;

            float hw = FRAMECENTER.X;
            roll = ((cp2.X / hw) - (cp1.X / hw)) * 90;

            float maxPixel = GetInchToPixel(center) * CameraProperties.objDepth;

            yaw = cur.CenterVector.X - center.CenterVector.X;
            yaw = yaw / maxPixel * 90;
            // this works also
            //yaw = cur.CenterVector.X - center.CenterVector.X;

            r = new Point3f(
                    pitch,
                    yaw,
                    roll
                );
        }

        public static float inchToPixel = 0f; // inches
        public static float GetInchToPixel(TShape center)
        {
            // so 1 inch is equal to x pixels. X is what we return
            return center.BaseVector.X / CameraProperties.objWidth;
        }
        public static void EstimateTransformation3(Mat frame, TShape center, TShape cur, UnitVector puv, UnitVector yuv, UnitVector ruv, out Point3f r, out Point3f t)
        {
            GetPureRotation(center, cur, out r);

            float expectedXT = center.TopCentroid.X + (r.Z * puv.Translation.X);
            float actualXT = cur.TopCentroid.X;

            float x = expectedXT - actualXT;

            float expectedYT = center.TopCentroid.Y + (r.Z * puv.Translation.Y);
            float actualYT = cur.TopCentroid.Y;

            float y = expectedYT - actualYT;

            r = new Point3f(r.X * puv.Rotation.X, r.Y * puv.Rotation.Y, r.Z * puv.Rotation.Z);

            // Translation
            t = new Point3f(
                x, y, 0
            );

        }

        public static void EstimateTransformation2(Mat frame, TShape center, TShape cur, out Point3f r, out Point3f t)
        {
            
            GetPureRotation(center, cur, out Point3f r2);

            float x = center.CenterVector.X - cur.CenterVector.X;
            float y = center.CenterVector.Y - cur.CenterVector.Y;


            // 
            r = r2;

            // Translation
            t = new Point3f(
                x, y, 0
            );
        }
        public static void DrawCircleAtPoint(Mat frame, Point2f point2Draw, Scalar sl, int r = 2)
        {
            Cv2.Circle(frame, point2Draw.R2P(), r, sl, 5);
        }

        public static Point2f Rotate(Point2f point, Point2f origin, double angleDegrees)
        {
            // Convert angle from degrees to radians
            double angleRadians = angleDegrees * Math.PI / 180.0;

            // Calculate the sine and cosine of the angle
            double cosTheta = Math.Cos(angleRadians);
            double sinTheta = Math.Sin(angleRadians);

            // Translate point back to origin
            float translatedX = point.X - origin.X;
            float translatedY = point.Y - origin.Y;
            translatedY = translatedY * -1;

            // Rotate point
            float rotatedX = (float)(translatedX * cosTheta - translatedY * sinTheta);
            float rotatedY = (float)(translatedX * sinTheta + translatedY * cosTheta);

            // Translate point back to its original position
            float finalX = rotatedX + origin.X;
            float finalY = rotatedY + origin.Y;

            return new Point2f(finalX, finalY);
        }

        public static double CalculateRotation(Point2f origin, Point2f point)
        {
            // Calculate the difference in coordinates
            float deltaX = point.X - origin.X;

            float deltaY = point.Y - origin.Y;
            deltaY = deltaY * -1;


            // Calculate the angle in radians using atan2
            double angleRadians = Math.Atan2(deltaY, deltaX);

            // Convert the angle to degrees
            double angleDegrees = angleRadians * (180.0 / Math.PI);

            // Ensure the angle is between -90 and 90 degrees

            //if(angleDegrees > 90)
            //{
            //    angleDegrees = angleDegrees * -1;
            //}
            //else
            //{
            //    angleDegrees = 180 - angleDegrees;
            //}

            return angleDegrees;
        }

        public static Point2f EstimateOrigin(Point2f centerVector, Point2f curVector)
        {
            return new Point2f();
        }

        public static Point2f AdjustMagnitude(Point2f cur, float desiredLength)
        {
            // Calculate the current magnitude of the vector
            float currentMagnitude = cur.Magnitude();

            // Avoid division by zero
            if (currentMagnitude == 0)
            {
                return new Point2f(0, 0);
            }

            // Calculate the scaling factor
            float scale = desiredLength / currentMagnitude;

            // Scale the vector components
            return new Point2f(cur.X * scale, cur.Y * scale);
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
