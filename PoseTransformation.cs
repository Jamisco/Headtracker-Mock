using OpenCvSharp;
using Emgu;
using System;
using System.IO;
using System.Linq;
using static Headtracker_Console.ExtensionMethods;
using static Headtracker_Console.HeadTracker;
using Emgu.CV.CvEnum;
using static Headtracker_Console.Calibrator;
using System.Collections.Generic;
using static OpenCvSharp.FileStorage;

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

        static Mat cameraMatrix = new Mat(3, 3, MatType.CV_64FC1);
        static Mat distCoeffs = Mat.Zeros(1, 5, MatType.CV_64FC1);

        public static Point3f tOffset = new Point3f();
        public static Point3f rOffset = new Point3f();

        private static Point2f[] centerPoints;

        static float depth = -100;
        public static void ResetPrediction()
        {
            rvecGuess = rvecGuess.EmptyClone();
            tvecGuess = rvecGuess.EmptyClone();

            //rvecGuess.Set<double>(0, 0, 0);
            //rvecGuess.Set<double>(1, 0, 0);
            //rvecGuess.Set<double>(2, 0, -8);

            //tvecGuess.Set<double>(0, 0, 0.1f);
            //tvecGuess.Set<double>(1, 0, 0.1f);
            //tvecGuess.Set<double>(2, 0, 10f);

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
        static float objectScaler = 10f;
        static Point3f initTranslation = new Point3f(0f, 0f, 30);

        public static void SetPrediction(Point2f[] points)
        {
            Point2f centriod = GetCentriod(points);

            // the z number doesnt matter that much, but it must be high

            tvecGuess.Set<double>(0, 0, initTranslation.X);
            tvecGuess.Set<double>(1, 0, initTranslation.Y);
            tvecGuess.Set<double>(2, 0, initTranslation.Z);

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
                if(TShape.IsValidPoint(projectedPoints.ToList()))
                {
                    TShape shape = new TShape(projectedPoints.ToList());
                    shape.DrawShape(frame, Scalar.Green, true);
                }
                else
                {
                    Cv2.PutText(frame, "Invalid Shape", new Point(80, 200), HersheyFonts.HersheyPlain, 5, Scalar.Red);
                }
            }
            else
            {
                Polygon shape = new Polygon(projectedPoints.ToList());
                shape.DrawShape(frame, Scalar.Green, true);
            }

            Cv2.ImShow("Projected Points", frame);
            Cv2.WaitKey(1);

        }

        public static void GetPureRoll(TShape center, TShape cur, out float roll)
        {
            Point2f cp1 = center.HeightVector;
            Point2f cp2 = cur.HeightVector;

            float hw = FRAMECENTER.X;
            roll = ((cp2.X / hw) - (cp1.X / hw)) * 90;
        }
        public static void GetPureRotation(TShape center, TShape cur, out Point3f r)
        {
            float pitch, yaw, roll;

            // the yaw angle can be derived by using the depth and get the max distance with the center can deviate from the center line. Said max distance is the max distance of the depth, or the depth from the base points to the back point

            Point2f cp1 = center.HeightVector;
            Point2f cp2 = cur.HeightVector;

            // im gonna assume this is account due to height being too short

            float curP = -(cur.TopCentroid.Y - cur.Centroid.Y);

            float centerP = -(center.TopCentroid.Y - center.Centroid.Y);

            float hbr = cur.Height / cur.Width;

            float chbr = center.Height / center.Width;

            pitch = (curP - centerP) / (centerP) * 45;

            float hw = FRAMECENTER.X;
            roll = ((cp2.X / hw) - (cp1.X / hw)) * 90;

            float maxPixel = GetInchToPixel(center) * CameraProperties.objDepth;

            yaw = cur.CenterVector.X - center.CenterVector.X;
            yaw = yaw / maxPixel * 90;
            // this works also
            //yaw = cur.CenterVector.X - center.CenterVector.X;

            pitch = pitch.RoundToInt();
            yaw = yaw.RoundToInt();
            roll = roll.RoundToInt();

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

        static Point line1 = new Point(0, 400);
        static Point line2 = new Point(0, 420);
        static Point line3 = new Point(0, 440);
        public static void EstimateTransformation3(Mat frame, TShape center, TShape cur, UnitVector puv, UnitVector yuv, UnitVector ruv, out Point3f r, out Point3f t)
        {
            GetPureRotation(center, cur, out r);

            Point2f expT = new Point2f();


            // when user zooms in or out , the translation is affected
            expT += ruv.ExpectedTranslation(r.Z);

            Point2f actualT = cur.Centroid - center.Centroid;

            float x = actualT.X - expT.X;

            float y = expT.Y - actualT.Y;

            Point3f offsetRotation = r;


            //puv.OffsetRotation(r, out r); 
            //yuv.OffsetRotation(r, out r);
            //ruv.OffsetRotation(r, out r);


            Cv2.PutText(frame, "Actual Translation: " + actualT.R2P(),
                        line1, HersheyFonts.HersheyPlain, 1, Scalar.White);

            Cv2.PutText(frame, "Expected Translation: " + expT.R2P(),
            line2, HersheyFonts.HersheyPlain, 1, Scalar.White);


            float maxPixel = GetInchToPixel(center) * CameraProperties.objDepth;

            float yaw = cur.CenterVector.X - center.CenterVector.X;
            yaw = Math.Abs(yaw / maxPixel);

            float yz = cur.CenterVector.Y - center.CenterVector.Y;

            float exWidth = cur.BaseVector.X + (cur.BaseVector.X * yaw) - yz;
            float axWidth = center.BaseVector.X;

            z = exWidth - axWidth;

            Cv2.PutText(frame, "Expected Width -- Diff: " + exWidth + " -- " + axWidth, line3, HersheyFonts.HersheyPlain, 1, Scalar.White);

            float div = 10;
            //x = x.RoundToDecimals() / div;
            //y = y.RoundToDecimals() / div;
            //z = z.RoundToDecimals() / div;

            // Translation
            t = new Point3f(
                x, y, -z
            );

        }

        public static void EstimateTransformation4(Mat frame, TShape center, TShape cur, Point2f od, out Point3f r, out Point3f t)
        {
            // the goal for this method will be to reconstruct the original triangle and thus from there decipher the transformation

            Point2f baseVectorDiff = cur.BaseVector - center.BaseVector;
            Point2f heightVectorDiff = cur.HeightVector - center.HeightVector;
            // flip base to get true yaw
            Point2f baseFlipped = new Point2f(-baseVectorDiff.Y, baseVectorDiff.X);

            Point2f vectorDiff = heightVectorDiff - baseFlipped;

            // projected points
            TShape projectShape = new TShape(cur.Points.ToList());

            //projectShape.AdjustBaseVector(baseVectorDiff, heightVectorDiff);
            //projectShape.TranslateTo(center.Centroid);

            projectShape.DrawShape(frame, Scalar.Green, false);

            FindOrigin(center, cur, out Point2f origin);

            tx += 10;
            Point2f tr = Rotate(cur.TopCentroid, origin, tx);

            DrawCircleAtPoint(frame, origin, Scalar.Red, 5);
            DrawCircleAtPoint(frame, tr, Scalar.Beige, 5);


            baseVectorDiff = baseVectorDiff.R2P(3);
            heightVectorDiff = heightVectorDiff.R2P(3);
            vectorDiff = vectorDiff.R2P(3);

            Cv2.PutText(frame, "Height Diff: " + heightVectorDiff.R2P(),
            line1, HersheyFonts.HersheyPlain, 1, Scalar.White);

            Cv2.PutText(frame, "Base Diff: " + baseVectorDiff.R2P(),
            line2, HersheyFonts.HersheyPlain, 1, Scalar.White);

            Cv2.PutText(frame, "Vector Diff: " + vectorDiff.R2P(),
            line3, HersheyFonts.HersheyPlain, 1, Scalar.White);

            float z = heightVectorDiff.X;

            float y = vectorDiff.X;

            float x = vectorDiff.Y;

            r = new Point3f(x, y, z);

            t = new Point3f();
        }

        public static void EstimateTransformation5(
                Mat frame,
                TShape shape,   // Current 2D points
                out Point3f r,               // Rotation angles
                out Point3f t)               // Translation vector
        {
            //ResetPrediction();
            Point3f[] objPoints = CameraProperties.objectPoints.ToArray();
            //Point3f[] objPoints = CameraProperties.SetObjectPointsFromCenter(curPoints);

            Point2f[] curPoints = shape.Points;

            // used to scale the object values if need me
            for (int i = 0; i < objPoints.Length; i++)
            {
                Point3f p = objPoints[i];

                objPoints[i] = p.Multiply(1 / objectScaler);
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

                string tDump = tvec.Dump();
                string rDump = rvec.Dump();

                //ProjectPoints(rvec, tvec);

                ExtractAngles(rvec, out r);
                ExtractTranslation(tvec, out t);

                Point2f start = new Point2f(300, 380);
                Point2f line = new Point2f(0, 20);
                int c = 0;

                Cv2.PutText(frame, "Rotation: " + r.R2P(3),
                        (start + line.Multiply(c++)).R2P(3), HersheyFonts.HersheyPlain, 1, Scalar.White);


                Cv2.PutText(frame, "Translation: " + t.R2P(3),
                        (start + line.Multiply(c++)).R2P(3), HersheyFonts.HersheyPlain, 1, Scalar.White);

                // Translation
                t = new Point3f(
                    t.X * objectScaler,
                    t.Y * objectScaler,
                    t.Z * (objectScaler / initTranslation.Z)
                );

                //t = t.Multiply(objectScaler);

                if (frameAfterCenter == 0)
                {
                    rOffset = r;
                    tOffset = t;
                }

                r = r - rOffset;
                t = t - tOffset;

                frameAfterCenter++;
            }
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

        public static void FindOrigin(TShape center, TShape cur, out Point2f origin)
        {
            // Ensure there are 3 points (for accurate calculation)
            if (center.Points.Length != 3 || cur.Points.Length != 3)
                throw new ArgumentException("Shapes must contain exactly 3 points.");

            // Calculate midpoint between corresponding points
            Point2f m1 = new Point2f((center.Points[0].X + cur.Points[0].X) / 2, (center.Points[0].Y + cur.Points[0].Y) / 2);
            Point2f m2 = new Point2f((center.Points[1].X + cur.Points[1].X) / 2, (center.Points[1].Y + cur.Points[1].Y) / 2);

            // Calculate perpendicular vectors
            Point2f v1 = new Point2f(cur.Points[0].Y - center.Points[0].Y, center.Points[0].X - cur.Points[0].X);
            Point2f v2 = new Point2f(cur.Points[1].Y - center.Points[1].Y, center.Points[1].X - cur.Points[1].X);

            // Solve intersection of the perpendicular bisectors
            float a1 = v1.Y;
            float b1 = -v1.X;
            float c1 = a1 * m1.X + b1 * m1.Y;

            float a2 = v2.Y;
            float b2 = -v2.X;
            float c2 = a2 * m2.X + b2 * m2.Y;

            float det = a1 * b2 - a2 * b1;
            if (Math.Abs(det) < 1e-6)
                throw new InvalidOperationException("Points are collinear; origin cannot be found.");

            // Calculate origin of rotation (intersection point)
            origin = new Point2f(
                (b2 * c1 - b1 * c2) / det,
                (a1 * c2 - a2 * c1) / det
            );
        }


        public static void ExtractAngles(Mat rvec, out Point3f rot)
        {
            Mat rotationMatrix = new Mat();
            Cv2.Rodrigues(rvec, rotationMatrix);

            // For yaw (Y rotation)
            double yaw = Math.Atan2(-rotationMatrix.Get<double>(2, 0),
            Math.Sqrt(rotationMatrix.Get<double>(2, 1) * rotationMatrix.Get<double>(2, 1) +
            rotationMatrix.Get<double>(2, 2) * rotationMatrix.Get<double>(2, 2)));

            // For roll (Z rotation)
            double roll = Math.Atan2(rotationMatrix.Get<double>(1, 0),
                                    rotationMatrix.Get<double>(0, 0));

            double pitch = Math.Atan2(rotationMatrix.Get<double>(2, 1),
                rotationMatrix.Get<double>(2, 2));

            rot = new Point3f(
                        (float)(pitch * 180.0 / Math.PI),
                        (float)(yaw * 180.0 / Math.PI),
                        (float)(roll * 180.0 / Math.PI)
            );
        }

        public static void ExtractTranslation(Mat tvec, out Point3f trans)
        {
            trans = new Point3f(
                (float)tvec.Get<double>(0, 0),
                (float)tvec.Get<double>(1, 0),
                (float)tvec.Get<double>(2, 0)
            );
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

        /// <summary>
        /// Given the object points, rotation vector, and translation vector, transform the original object points to the current points.
        /// </summary>
        /// <param name="objectPoints"></param>
        /// <param name="rvec"></param>
        /// <param name="tvec"></param>
        /// <returns></returns>
        public static List<Point3f> TransformPoint(Point3f[] objectPoints, Mat rvec, Mat tvec)
        {
            List<Point3f> tPoints = new List<Point3f>();

            foreach (Point3f original in objectPoints)
            {
                // Convert the rotation vector to a rotation matrix
                Mat rmat = new Mat();
                Cv2.Rodrigues(rvec, rmat);

                // Convert Point3f to a Mat for matrix multiplication
                Mat pointMat = new Mat(3, 1, MatType.CV_64F); // Create an uninitialized Mat
                pointMat.Set<double>(0, 0, original.X); // Set the value at (0, 0) to original.X
                pointMat.Set<double>(1, 0, original.Y); // Set the value at (1, 0) to original.Y
                pointMat.Set<double>(2, 0, original.Z); // Set the value at (2, 0) to original.Z

                // Apply rotation: R * P
                Mat rotatedPoint = rmat * pointMat;

                string pdump = pointMat.Dump();
                string rmDump = rmat.Dump();
                string rdump = rotatedPoint.Dump();
                string tdump = tvec.Dump();

                // Apply translation: R * P + T
                Point3f transformedPoint = new Point3f(
                    (float)(rotatedPoint.At<double>(0, 0) + tvec.At<double>(0, 0)),
                    (float)(rotatedPoint.At<double>(1, 0) + tvec.At<double>(1, 0)),
                    (float)(rotatedPoint.At<double>(2, 0) + tvec.At<double>(2, 0))
                );

                tPoints.Add(transformedPoint);
            }

            return tPoints;
        }

        public static void EstimateRigidTransform(List<Point3f> original, List<Point3f> transformed, out Mat rvec, out Mat tvec)
        {
            if (original.Count != transformed.Count || original.Count < 3)
                throw new ArgumentException("At least 3 corresponding points are required.");

            // Convert lists to Mats
            Mat originalMat = new Mat(original.Count, 3, MatType.CV_64F);
            Mat transformedMat = new Mat(transformed.Count, 3, MatType.CV_64F);

            for (int i = 0; i < original.Count; i++)
            {
                originalMat.Set<double>(i, 0, original[i].X);
                originalMat.Set<double>(i, 1, original[i].Y);
                originalMat.Set<double>(i, 2, original[i].Z);

                transformedMat.Set<double>(i, 0, transformed[i].X);
                transformedMat.Set<double>(i, 1, transformed[i].Y);
                transformedMat.Set<double>(i, 2, transformed[i].Z);
            }

            string odUMP = originalMat.Dump();
            string tDump = transformedMat.Dump();

            // Calculate centroids
            Point3f centroidOriginal = new Point3f(
                (float)originalMat.Col(0).Mean().Val0,
                (float)originalMat.Col(1).Mean().Val0,
                (float)originalMat.Col(2).Mean().Val0
            );

            Point3f centroidTransformed = new Point3f(
                (float)transformedMat.Col(0).Mean().Val0,
                (float)transformedMat.Col(1).Mean().Val0,
                (float)transformedMat.Col(2).Mean().Val0
            );

            // Center the points around their centroid
            for (int i = 0; i < original.Count; i++)
            {
                original[i] -= centroidOriginal;
                transformed[i] -= centroidTransformed;
            }

            // Convert centered points to Mats
            Mat centeredOriginal = new Mat(original.Count, 3, MatType.CV_64F);
            Mat centeredTransformed = new Mat(transformed.Count, 3, MatType.CV_64F);

            for (int i = 0; i < original.Count; i++)
            {
                centeredOriginal.Set<double>(i, 0, original[i].X);
                centeredOriginal.Set<double>(i, 1, original[i].Y);
                centeredOriginal.Set<double>(i, 2, original[i].Z);

                centeredTransformed.Set<double>(i, 0, transformed[i].X);
                centeredTransformed.Set<double>(i, 1, transformed[i].Y);
                centeredTransformed.Set<double>(i, 2, transformed[i].Z);
            }

            // Compute cross-covariance matrix
            Mat H = centeredOriginal.T() * centeredTransformed;

            // Perform SVD on H
            Mat w = new Mat(), u = new Mat(), vt = new Mat();
            Cv2.SVDecomp(H, w, u, vt);

            // Compute rotation matrix: R = V * U^T
            Mat R = vt.T() * u.T();

            // Ensure proper orientation (det(R) = 1 for a valid rotation matrix)
            if (Cv2.Determinant(R) < 0)
            {
                Mat vtRow2 = vt.Row(2);
                vtRow2 *= -1;
                R = vt.T() * u.T();
            }

            // Convert rotation matrix to rotation vector
            rvec = new Mat();
            Cv2.Rodrigues(R, rvec);

            // Compute translation: T = centroidTransformed - R * centroidOriginal
            Mat centroidOriginalMat = new Mat(3, 1, MatType.CV_64F);
            centroidOriginalMat.Set<double>(0, 0, centroidOriginal.X);
            centroidOriginalMat.Set<double>(1, 0, centroidOriginal.Y);
            centroidOriginalMat.Set<double>(2, 0, centroidOriginal.Z);

            Mat rotatedCentroid = R * centroidOriginalMat;
            tvec = new Mat(3, 1, MatType.CV_64F);

            tvec.Set<double>(0, 0, centroidTransformed.X - rotatedCentroid.At<double>(0, 0));
            tvec.Set<double>(1, 0, centroidTransformed.Y - rotatedCentroid.At<double>(1, 0));
            tvec.Set<double>(2, 0, centroidTransformed.Z - rotatedCentroid.At<double>(2, 0));
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
            frameAfterCenter = 0;

        }

        static float z = 50;



        public static void ClearOffsets()
        {
            frameAfterCenter = 0;
        }
    }
}
