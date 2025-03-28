using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using static Headtracker_Console.CameraProperties;
using static Headtracker_Console.ExtensionMethods;

namespace Headtracker_Console
{
    public static class PoseTransformation
    {
        static string matStr = "C:\\Users\\mufuh\\source\\repos\\Headtracker_Console\\mtxNdist.txt";
        // Initial guess for rotation and translation
        static Mat rvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);
        static Mat tvecGuess = Mat.Zeros(3, 1, MatType.CV_64FC1);

        public static Point3f tOffset = new Point3f();
        public static Point3f rOffset = new Point3f();

        static bool setOffset = false;
        static bool initFrame = true;

        static PoseTransformation()
        {
            //LoadCalibrationData();
        }
        public static void ResetPrediction()
        {
            rvecGuess = rvecGuess.EmptyClone();
            tvecGuess = rvecGuess.EmptyClone();
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

        static float objectScaler = 10f;
        static Point3f initTranslation = new Point3f(0f, 0f, 30);
        public static void SetPrediction()
        {

            // the z number doesnt matter that much, but it must be high

            tvecGuess.Set<double>(0, 0, initTranslation.X);
            tvecGuess.Set<double>(1, 0, initTranslation.Y);
            tvecGuess.Set<double>(2, 0, initTranslation.Z);

            rvecGuess.Set<double>(0, 0, 0);
            rvecGuess.Set<double>(1, 0, 0);
            rvecGuess.Set<double>(2, 0, 0);
        }

        public static void QuickEstimate(
                Mat frame,
                TShape shape,   // Current 2D points
                out Point3f r) 
        {

            List<Point2f> leds = new List<Point2f>();

            foreach (var p in CameraProperties.objectPoints)
            {
                leds.Add(new Point2f(p.X, p.Y));
            }

            TShape center = new TShape(leds);

            float cs = shape.Height + shape.Width;
            float cc = center.Height + center.Width;

            float pitch = (float)Math.Sin(Math.PI * ((shape.Height / cs) - (center.Height / cc)));

            if(pitch < 0)
            {
                pitch = pitch * 60;
            }
            else
            {
                pitch = pitch * 180;
            }

            float yaw = shape.Centroid.X - shape.TopCentroid.X - center.Centroid.X + center.TopCentroid.X;

            r = new Point3f();

            Cv2.PutText(frame, "Pitch: " + pitch,
                        new Point(300, 420), HersheyFonts.HersheyPlain, 1, Scalar.White);

            r = new Point3f(pitch, 0, 0);


        }

        public static void AutoAdjustCenter(Mat frame, TShape shape, TShape center = null)
        {
            Point2f screenCenter = new Point2f(frame.Width / 2, frame.Height / 2);
            string rr = "Ressetting ";
            float cr = 20;

            float distance = (float)Math.Abs(shape.Centroid.X - screenCenter.X);

            float x = (float)tvecGuess.Get<double>(0, 0);
            x = Math.Abs(x) * objectScaler;

            if (distance < cr && x > 1)
            {
                tvecGuess.Set<double>(0, 0, 0);
                rvecGuess.Set<double>(1, 0, 0);
                rr += " X";
            }

            distance = (float)Math.Abs(shape.Centroid.Y - screenCenter.Y);
            float y = (float)tvecGuess.Get<double>(1, 0);
            y = Math.Abs(y) * objectScaler;


            if (distance < cr && y > 1)
            {
                tvecGuess.Set<double>(1, 0, 0);
                rvecGuess.Set<double>(0, 0, 0);

                rr += " Y";
            }

            if(center != null)
            {
                float cd = (float)Point2f.Distance(shape.Centroid, center.Centroid);

                float cp = shape.Height + shape.Width;
                float cc = center.Height + center.Width;

                float pp = (cp / cc) * 100;

                if (cd < cr && pp < 10)
                {
                    tvecGuess.Set<double>(2, 0, initTranslation.Z);
                    rr += " Z";
                }

                Cv2.Circle(frame, center.Centroid.R2P(), 20, Scalar.Green, 2);

            }

            Cv2.PutText(frame, rr, new Point(300, 460), HersheyFonts.HersheyPlain, 1, Scalar.White);
        }

        public static TShape centerShape = null;
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

            if(initFrame)
            {
                SetPrediction();
                initFrame = false;
            }

            string rd = rvecGuess.Dump();
            string td = tvecGuess.Dump();
            string cam = cameraMatrix.Dump();
            string ddd = distCoeffs.Dump();

            AutoAdjustCenter(frame, shape, centerShape);

        SolvePnP:
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
                QuickEstimate(frame, shape, out Point3f rt);

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
                    -t.X * objectScaler,
                    -t.Y * objectScaler,
                    t.Z * (objectScaler / initTranslation.Z)
                );

                t = t.Multiply(10);

                //t = t.Multiply(objectScaler);

                if (setOffset == true)
                {
                    rOffset = r;
                    tOffset = t;
                    centerShape = shape;
                    setOffset = false;
                }

                r = r - rOffset;
                t = t - tOffset;

            }
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
                        -(float)(yaw * 180.0 / Math.PI),
                        -(float)(roll * 180.0 / Math.PI)
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
        public static void ClearOffsets()
        {
            ResetPrediction();
            SetPrediction();

            setOffset = true;

        }
    }
}
