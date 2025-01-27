using Headtracker_Console;
using OpenCvSharp;
using ScottPlot.Colormaps;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using static Headtracker_Console.Calibration;


public class HeadTracker
{
    private VideoCapture capture;
    private Mat frame = new Mat();

    private Thread trackingThread;
    private Thread keyPressThread;

    const int FRAMEHEIGHT = 800;
    const int FRAMEWIDTH = 800;
    public const int CaptureFps = 30;
    private bool isTracking;

    public enum Transformations { Scale, Yaw, Roll, Pitch, X, Y, Z };

    public HeadTracker(int cameraIndex = 0)
    {
        // Initialize the camera
        capture = new VideoCapture(cameraIndex);
        capture.Set(VideoCaptureProperties.Fps, CaptureFps); // Adjust as needed
        capture.FrameHeight = 800;
        capture.FrameWidth = 800;
        Console.WriteLine("Size: " + capture.FrameHeight + " - " + capture.FrameWidth);
    }

    // Detect contours or blobs (these would correspond to your LEDs/markers)
    Point[][] curContours;
    public Triangle curTriangle { get; private set; }
    public Triangle centerTriangle { get; private set; }
    public bool HasCenter { get; set; } = false;

    Calibration calibration;
    public void StartTracking()
    {
        Console.WriteLine("Press a key to Begin...");

        Console.ReadKey();

        isTracking = true;
        trackingThread = new Thread(TrackingLoop);
        keyPressThread = new Thread(ReadKey);
        calibration = new Calibration(this);

        trackingThread.Start();
        keyPressThread.Start();
    }

    public int frameCount = 0;

    public void ReadKey()
    {
        while (true)
        {
            if (Console.KeyAvailable)
            {
                ConsoleKeyInfo k = Console.ReadKey();

                if (k.Key == ConsoleKey.Spacebar)
                {
                    HasCenter = true;
                    centerTriangle = curTriangle;
                    calibration.UpdateCenter();
                    Console.WriteLine("New Center Set");
                }

                if (k.Key == ConsoleKey.X)
                {
                    if(!HasCenter)
                    {
                        Console.WriteLine("Set Center Before Calibration");
                    }
                    else
                    {
                        calibration.IsCalibrating = !calibration.IsCalibrating;

                        if(calibration.IsCalibrating)
                        {
                            Console.WriteLine("Started Calibration");
                        }
                        else
                        {
                            Console.WriteLine("Stopped Calibration");
                            calibration.CalibrateOffsets();
                        }
                    }
                }

                if (k.Key == ConsoleKey.Q && calibration.IsCalibrating)
                {
                    calibration.CalibratePitch = !calibration.CalibratePitch;
                    
                    if(calibration.CalibratePitch)
                    {
                        Console.WriteLine("Started Pitch Calibration");
                    }
                    else
                    {
                        Console.WriteLine("Stopped Pitch Calibration");
                    }
                }

                if (k.Key == ConsoleKey.W && calibration.IsCalibrating)
                {
                    calibration.CalibrateYaw = !calibration.CalibrateYaw;

                    if (calibration.CalibrateYaw )
                    {
                        Console.WriteLine("Started Yaw Calibration");
                    }
                    else
                    {
                        Console.WriteLine("Stopped Yaw Calibration");
                    }
                }

                if (k.Key == ConsoleKey.E && calibration.IsCalibrating)
                {
                    calibration.CalibrateRoll = !calibration.CalibrateRoll;

                    if (calibration.CalibrateRoll)
                    {
                        Console.WriteLine("Started Roll Calibration");
                    }
                    else
                    {
                        Console.WriteLine("Stopped Roll Calibration");
                    }
                }



                if (k.Key == ConsoleKey.C && calibration.IsCalibrating)
                {
                    calibration.ClearCurrentPoints();
                    Console.WriteLine("Calibration Cleared");
                }

                if (k.Key == ConsoleKey.S)
                {
                    calibration.Serialize();
                    Console.WriteLine("Calibration Saved");
                }
            }
        }
    }

    Mat displayFrame;

    string frameName = "Tracking Frame";

    private void TrackingLoop()
    {
        Mat prevFrame = new Mat();
        // calibration frame completed, now we simply need to show the calibration points as we are calibrating or just skip straight to calculation
        while (isTracking)
        {
            capture.Read(frame);

            if (frame.Empty() || frame.IsThesame(prevFrame))
            {
                continue;
            }

            ExtractCurrentTriangle(frame);

            displayFrame = frame.EmptyClone();

            if(!calibration.IsCalibrating)
            {
                if(!curTriangle.IsValid)
                {
                    goto Skip;
                }

                ShowCenterTriangle(displayFrame);
                ShowCurrentTriangle(displayFrame);
                ShowCalculatedTriangle(displayFrame);
                ShowFrameCounter(displayFrame);
            }
            else
            {
                centerTriangle.DrawGraph(displayFrame);
                curTriangle.DrawCentriod(displayFrame);

                calibration.AddCalibrationPoint(ref displayFrame);
            }

            Skip:

            Cv2.NamedWindow(frameName);
            Cv2.SetWindowProperty(frameName, WindowPropertyFlags.Fullscreen, 5);
            Cv2.ImShow(frameName, displayFrame);
            Cv2.WaitKey(1);

            frameCount++;
            prevFrame = frame.Clone();
        }
    }

    Point2f start = new Point2f(10, 20);
    Point2f offset = new Point2f(0, 150);
    private void ExtractCurrentTriangle(Mat frame)
    {
        try
        {
            Mat grayFrame = new Mat();

            Cv2.CvtColor(frame, grayFrame, ColorConversionCodes.BGR2GRAY);

            HierarchyIndex[] hierarchy;

            // apply gausasain filter
            Cv2.GaussianBlur(grayFrame, grayFrame, new Size(5, 5), 0);

            Cv2.Threshold(grayFrame, grayFrame, 50, 255, ThresholdTypes.Binary);

            Cv2.FindContours(grayFrame, out curContours, out hierarchy, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

            GetLedContours(ref curContours);

            Triangle temp;
            GetTrianglePoints(curContours, out temp);

            curTriangle = temp;
        }
        catch (Exception)
        {

            curTriangle = new Triangle();
        }

    }
    private void ShowCurrentTriangle(Mat displayFrame)
    {
        try
        {
            curTriangle.DrawTriangle(displayFrame, Scalar.Red);
            curTriangle.DrawCentriod(displayFrame);
            curTriangle.PrintData(displayFrame, start);

        }
        catch (Exception)
        {

        }
    }
    private void ShowCenterTriangle(Mat displayFrame)
    {
        Mat f = frame.EmptyClone();

        if (HasCenter)
        {
            centerTriangle.DrawTriangle(displayFrame, Scalar.White, true);
        }
    }

    private void ShowFrameCounter(Mat displayFrame)
    {
        Point bottom = new Point(0, 580);

        Cv2.PutText(displayFrame, "Frame Count: " + frameCount, bottom, HersheyFonts.HersheyPlain, 1, Scalar.White);
    }

    private void ShowCalculatedTriangle(Mat displayFrame)
    {
        if (calibration.FullyCalibrated)
        {
            //calibration.GetTransformation(centerTriangle, curTriangle, out Triangle t2, out float yaw, out float pitch);

            calibration.GetTransformation2(centerTriangle, curTriangle, out Triangle t2);

            t2.DrawTriangle(displayFrame, Scalar.Green, true);
            t2.PrintData(displayFrame, start + offset);
        }
    }

    private void ShowTestedTriangle(Mat displayFrame)
    {
        if (calibration.FullyCalibrated && HasCenter)
        {
            calibration.TestAngle(centerTriangle, out Triangle t2);

            t2.DrawTriangle(displayFrame, Scalar.Blue, true);
        }
    }

    public void GetLedContours(ref Point[][] contours)
    {
        if (contours.Length < 2)
        {
            return;
        }

        // Step 1: Calculate areas for all contours
        List<Tuple<Point[], double>> contourAreas = new List<Tuple<Point[], double>>();

        foreach (var contour in contours)
        {
            double area = Cv2.ContourArea(contour);
            contourAreas.Add(Tuple.Create(contour, area));
        }

        // Step 2: Sort the contours by area (descending order)
        contourAreas.Sort((a, b) => b.Item2.CompareTo(a.Item2));

        // Step 3: Take the two largest contours (if there are at least 2)
        if (contourAreas.Count >= 2)
        {
            // Limit to a maximum of 3 contours
            int cc = contourAreas.Count > 3 ? 3 : contourAreas.Count;

            contours = contourAreas.Take(cc).ToList().Select(t => t.Item1).ToArray();
        }
    }
    private void GetTrianglePoints(Point[][] contours, out Triangle triangle)
    {
        // the top Point2f is index 0
        // the right Point2f is index 1
        // the left Point2f is index 2
        // the centroid is index 3

        List<Point2f> triPoint2fs = new List<Point2f>();

        for (int i = 0; i < 3; i++)
        {
            Moments moments = Cv2.Moments(contours[i]);
            triPoint2fs.Add(new Point2f((int)(moments.M10 / moments.M00), (int)(moments.M01 / moments.M00)));
        }

        triPoint2fs.Sort((a, b) => a.Y.CompareTo(b.Y));

        if (triPoint2fs[1].X < triPoint2fs[2].X)
        {
            Point2f temp = triPoint2fs[1];
            triPoint2fs[1] = triPoint2fs[2];
            triPoint2fs[2] = temp;
        }

        triangle = new Triangle(triPoint2fs);
    }
    public enum Axis { XAxis, YAxis, ZAxis };

    [Serializable]
    public struct TriangleDiff
    {
        public Point2f centriodDiff;
        public Point2f[] vectorDiff;
        public TriangleDiff(Triangle t1, Triangle t2)
        {
            Point2f[] cv = new Point2f[3];

            centriodDiff = (t2.Centroid - t1.Centroid);

            Point2f[] v1 = t1.Vectors;
            Point2f[] v2 = t2.Vectors;

            for (int i = 0; i < 3; i++)
            {
                cv[i] = v2[i] - v1[i];
            }

            vectorDiff = cv;
        }

        public static TriangleDiff operator +(TriangleDiff t1, TriangleDiff t2)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f centriodDiff = new Point2f();

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t1.vectorDiff[i] + t2.vectorDiff[i];
            }

            centriodDiff = t1.centriodDiff + t2.centriodDiff;

            return new TriangleDiff()
            {
                vectorDiff = vectorDiffs,
                centriodDiff = centriodDiff,

            };
        }
        public static TriangleDiff operator -(TriangleDiff t1, TriangleDiff t2)
        {
            return t1 + (t2 * -1);
        }
        public static TriangleDiff operator *(TriangleDiff t1, float num)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f centriodDiff = new Point2f();

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t1.vectorDiff[i] * num;
            }

            centriodDiff = t1.centriodDiff * num;

            return new TriangleDiff()
            {
                centriodDiff = centriodDiff,
                vectorDiff = vectorDiffs,
            };

        }
        public static TriangleDiff operator /(TriangleDiff t1, float div)
        {
            return t1 * (1 / div);
        }
    }

    [Serializable]
    public struct Triangle
    {
        public Point2f top;
        public Point2f left;
        public Point2f right;

        public Point2f Centroid => new Point2f((top.X + left.X + right.X) / 3, (top.Y + left.Y + right.Y) / 3);

        public float Top2Left => (float)Point2f.Distance(top, left);
        public float Top2Right => (float)Point2f.Distance(top, right);
        public float Left2Right => (float)Point2f.Distance(left, right);

        // check if the points are within frame height and width
        public bool IsValid
        {
            get
            {
                foreach (var point in Points)
                {
                    if (point.X < 0 || point.X >= FRAMEWIDTH || point.Y < 0 || point.Y >= FRAMEHEIGHT)
                    {
                        return false;
                    }
                }

                // add up angles to make sure its within 180

                bool hasBadAngle = Angles.Any(a => float.IsNaN(a));

                if(hasBadAngle)
                {
                    return false;
                }

                float angles = Angles[0] + Angles[1] + Angles[2];

                angles = Math.Abs(angles - 180);

                // a triangle should have angle of size 180,
                // we afford tolerance of 5 degrees, any more, angles isnt valid
                if (angles > 5)
                {
                    return false;
                }

                return true;
            }
        }
        public float[] Angles => new float[] { LeftAngle, TopAngle, RightAngle };
        public float LeftAngle
        {
            get
            {
                float a = Top2Right;
                float b = Top2Left;
                float c = Left2Right;
                return (float)(Math.Acos((b * b + c * c - a * a) / (2 * b * c)) * (180 / Math.PI));
            }
        }

        public float RightAngle
        {
            get
            {
                float a = Top2Right;
                float b = Top2Left;
                float c = Left2Right;
                return (float)(Math.Acos((a * a + c * c - b * b) / (2 * a * c)) * (180 / Math.PI));
            }
        }

        public float TopAngle
        {
            get
            {
                float a = Top2Right;
                float b = Top2Left;
                float c = Left2Right;
                return (float)(Math.Acos((a * a + b * b - c * c) / (2 * a * b)) * (180 / Math.PI));
            }
        }

        private float Distance(Point2f p1, Point2f p2)
        {
            return (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }

        //public Mat PointsMatrix
        //{
        //    get
        //    {
        //        Mat m = new Mat(3, 2, MatType.CV_32FC1);

        //        m.Set<float>(0, 0, top.X);
        //        m.Set<float>(0, 1, top.Y);

        //        m.Set<float>(1, 0, right.X);
        //        m.Set<float>(1, 1, right.Y);

        //        m.Set<float>(2, 0, left.X);
        //        m.Set<float>(2, 1, left.Y);

        //        return m;
        //    }
        //}

        public float Perimeter
        {
            get
            {
                return Top2Left + Top2Right + Left2Right;
            }
        }

        public float[] SideLengths
        {
            get
            {
                return new float[] { Top2Left, Top2Right, Left2Right };
            }
        }

        public Point2f[] Points
        {
            get
            {
                Point2f[] Point2fs = new Point2f[3];

                Point2fs[0] = top;
                Point2fs[1] = right;
                Point2fs[2] = left;

                return Point2fs;
            }
        }
        public Point2f[] Vectors
        {
            get
            {
                Point2f[] vectors = new Point2f[3];

                // left to top
                vectors[0] = top - left;

                // left to right
                vectors[1] = right - left;

                // right to top
                vectors[2] = top - right;

                return vectors;
            }
        }
        public Triangle(List<Point2f> cur)
        {
            this.top = cur[0];
            this.right = cur[1];
            this.left = cur[2];
        }

        /// <summary>
        /// Create triangle from 3 x 2 matrix
        /// </summary>
        /// <param name="matrix32"></param>
        public Triangle(Mat matrix32)
        {
            if (matrix32.Rows != 3 || matrix32.Cols != 2)
                throw new ArgumentException("Matrix must be 3x2.");

            top = new Point2f(matrix32.At<float>(0, 0), matrix32.At<float>(0, 1));
            right = new Point2f(matrix32.At<float>(1, 0), matrix32.At<float>(1, 1));
            left = new Point2f(matrix32.At<float>(2, 0), matrix32.At<float>(2, 1));
        }

        public static Triangle FromVectors(Point2f[] vectors, Point2f leftPoint)
        {
            Point2f[] points = new Point2f[3];

            points[0] = leftPoint + vectors[0];
            points[1] = leftPoint + vectors[1];

            points[2] = leftPoint;

            return new Triangle(points.ToList());
        }
        private static Point2f GetVector(Point2f a, Point2f b)
        {
            return new Point2f(b.X - a.X, b.Y - a.Y);
        }

        public static (float scale, float yaw, float roll, float pitch, float distance) Compare(Triangle t1, Triangle t2)
        {
            float pitch = 1 - (t2.Top2Left + t2.Top2Right) / (t1.Top2Left + t1.Top2Right);

            float scale = 1 - (t2.Perimeter / t1.Perimeter);

            float yaw = 1 - (t2.Left2Right / t1.Left2Right);

            float hDiff1 = t1.left.Y - t1.right.Y;
            float hDiff2 = t2.left.Y - t2.right.Y;

            float roll = ((hDiff2 - hDiff1) / t1.Left2Right);

            float distance = (float)Point2f.Distance(t1.Centroid, t2.Centroid);

            scale *= 100;
            yaw *= 100;
            roll *= 100;
            pitch *= 100;

            return (scale, yaw, roll, pitch, distance);

        }
        public void Scale(float scaleFactor)
        {
            top = ((top - Centroid) * scaleFactor) + Centroid;
            right = ((right - Centroid) * scaleFactor) + Centroid;
            left = ((left - Centroid) * scaleFactor) + Centroid;
        }
        public void Translate(Point2f p)
        {
            top -= p;
            left -= p;
            right -= p;
        }
        public void TranslateTo(Point2f center)
        {
            // move current triangle to new center

            Point2f cDiff = center - Centroid;

            top += cDiff;
            left += cDiff;
            right += cDiff;
        }
        private void LabelPoint2fs(Mat frame, List<Point2f> Point2fs, int col = 0)
        {
            Point2f dp = new Point2f(50, 0);
            Point2f tp = new Point2f(0, 50);

            Scalar scal = Scalar.White;

            if (col != 0)
            {
                scal = Scalar.Red;
            }

            string top = "(" + Point2fs[0].X + ", " + Point2fs[0].Y + ")";
            string right = "(" + Point2fs[1].X + ", " + Point2fs[1].Y + ")";
            string left = "(" + Point2fs[2].X + ", " + Point2fs[2].Y + ")";

            Cv2.PutText(frame, top, (Point2fs[0] + dp).R2P(), HersheyFonts.HersheyPlain, 1, scal);

            Cv2.PutText(frame, right, (Point2fs[1] + dp).R2P(), HersheyFonts.HersheyPlain, 1, scal);

            Cv2.PutText(frame, left, (Point2fs[2] + tp).R2P(), HersheyFonts.HersheyPlain, 1, scal);
        }
        private void DrawCircle(Mat frame, List<Point2f> Point2fs)
        {
            Scalar scal = Scalar.White;

            Cv2.Circle(frame, Point2fs[0].R2P(), 5, scal, 2);
            Cv2.Circle(frame, Point2fs[1].R2P(), 5, scal, 2);
            Cv2.Circle(frame, Point2fs[2].R2P(), 5, scal, 2);
        }
        public void DrawTriangle(Mat frame, Scalar sl, bool showLengths = true)
        {
            Point2f px = new Point2f(20, 0);
            Point2f xp = new Point2f(70, 0);
            Point2f py = new Point2f(0, 30);

            //Scalar sl = Scalar.White;

            Cv2.Circle(frame, top.R2P(), 2, sl, 2);
            Cv2.Circle(frame, left.R2P(), 2, sl, 2);
            Cv2.Circle(frame, right.R2P(), 2, sl, 2);
            Cv2.Circle(frame, Centroid.R2P(), 2, sl, 2);

            // Draw lines and shows their distances

            Cv2.Line(frame, top.R2P(), left.R2P(), sl, 2);
            Cv2.Line(frame, top.R2P(), right.R2P(), sl, 2);
            Cv2.Line(frame, left.R2P(), right.R2P(), sl, 2);

            if (showLengths)
            {
                Cv2.PutText(frame, Top2Left.ToString("F2"), (MidPoint2f(top, left) - xp).R2P(), HersheyFonts.HersheyPlain, 1, sl);

                Cv2.PutText(frame, Top2Right.ToString("F2"), (MidPoint2f(top, right) + px).R2P(), HersheyFonts.HersheyPlain, 1, sl);

                Cv2.PutText(frame, Left2Right.ToString("F2"), (MidPoint2f(left, right) + py).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            }

            // labels Point2f
            //string top1 = "(" + top.X + ", " + top.Y + ")";
            //string right1 = "(" + right.X + ", " + right.Y + ")";
            //string left1 = "(" + left.X + ", " + left.Y + ")";

            //Cv2.PutText(frame, top1, (top - py).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            //Cv2.PutText(frame, right1, (right + px).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            //Cv2.PutText(frame, left1, ((left - xp + py).R2P()), HersheyFonts.HersheyPlain, 1, sl);
        }
        public void DrawCentriod(Mat frame)
        {
            Cv2.Circle(frame, Centroid.R2P(), 2, Scalar.White, 2);
        }

        public void DrawGraph(Mat frame)
        {
            int circleCount = 15;
            int ra = 10;

            int height = frame.Height;
            int width = frame.Width;

            int hs = height / circleCount;
            int ws = width / circleCount;

            int x = (int)Centroid.X;
            int y = (int)Centroid.Y;


            Point x1 = new Point(0, y);
            Point x2 = new Point(width, y);

            Point y1 = new Point(x, 0);
            Point y2 = new Point(x, height);

            Cv2.Line(frame, x1, x2, Scalar.White, 1);
            Cv2.Line(frame, y1, y2, Scalar.White, 1);

            Cv2.Circle(frame, Centroid.R2P(), ra, Scalar.White, 2);

            for (int i = 0; i < circleCount; i++)
            {
                Point xp = new Point(x + (i * ws), y);
                Point nxp = new Point(x - (i * ws), y);

                Point yp = new Point(x, y + (i * hs));
                Point nyp = new Point(x, y - (i * hs));

                Cv2.Circle(frame, xp, ra, Scalar.Red);
                Cv2.Circle(frame, nxp, ra, Scalar.Red);

                Cv2.Circle(frame, yp, ra, Scalar.Red);
                Cv2.Circle(frame, nyp, ra, Scalar.Red);
            }

        }

        static Point2f PrintGap = new Point2f(0, 50);
        public void PrintData(Mat frame, Point2f pos)
        {
            // print the center location and the lengths

            Point2f px = new Point2f(20, 0);
            Point2f py = new Point2f(0, 20);

            Scalar sl = Scalar.White;

            int c = 0;

            Cv2.PutText(frame, "Centroid: " + Centroid.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            //Cv2.PutText(frame, "Top: " + top.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            //Cv2.PutText(frame, "Right: " + right.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            //Cv2.PutText(frame, "Left: " + left.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Top2Left: " + Top2Left.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            Cv2.PutText(frame, "Top2Right: " + Top2Right.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            Cv2.PutText(frame, "Left2Right: " + Left2Right.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Point2f[] vs = Vectors;

            //Cv2.PutText(frame, "Top2Left V: " + vs[0].ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            //Cv2.PutText(frame, "Top2Right V: " + vs[1].ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            //Cv2.PutText(frame, "Left2Right V: " + vs[2].ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);


            // print angles

            Cv2.PutText(frame, "Top Angle: " + TopAngle.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            Cv2.PutText(frame, "Left Angle: " + LeftAngle.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Right Angle: " + RightAngle.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

        }
        private Point2f GetTranslation(Triangle o, Triangle m)
        {
            Point2f translation = new Point2f();

            translation.X = m.Centroid.X - o.Centroid.X;
            translation.Y = m.Centroid.Y - o.Centroid.Y;

            return translation;
        }
        public override bool Equals(object obj)
        {
            if (obj.GetType() == typeof(Triangle))
            {
                Triangle t2 = (Triangle)obj;

                return top == t2.top && right == t2.right && left == t2.left;
            }
            else
            {
                return false;
            }
        }
        public static bool operator ==(Triangle t1, Triangle t2)
        {
            return t1.Equals(t2);
        }

        public static bool operator !=(Triangle t1, Triangle t2)
        {
            return !t1.Equals(t2);
        }

        public static Triangle operator +(Triangle t1, Triangle t2)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f leftPoint = t1.left;

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t2.Vectors[i] + t1.Vectors[i];
            }

            Triangle nt = FromVectors(vectorDiffs, leftPoint);


            return nt;
        }

        public static Triangle operator +(Triangle t1, TriangleDiff t2)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f leftPoint = t1.left;

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t1.Vectors[i] + t2.vectorDiff[i];
            }

            Point2f center = t1.Centroid + t2.centriodDiff;

            Triangle nt = FromVectors(vectorDiffs, leftPoint);

            nt.TranslateTo(center);

            return nt;
        }
        public static Triangle operator *(Triangle t1, float t2)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f leftPoint = t1.left;
            Point2f center = t1.Centroid;

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t1.Vectors[i] * t2;
            }

            Triangle nt = FromVectors(vectorDiffs, leftPoint);

            nt.TranslateTo(center);

            return nt;
        }

        public static Triangle Square(Triangle t1)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f leftPoint = t1.left;
            Point2f center = t1.Centroid;

            for (int i = 0; i < 3; i++)
            {
                float x = t1.Vectors[i].X;
                float y = t1.Vectors[i].Y;

                vectorDiffs[i] = new Point2f(x * x, y * y);
            }

            Triangle nt = FromVectors(vectorDiffs, leftPoint);

            center = new Point2f(center.X * center.X, center.Y * center.Y);

            nt.TranslateTo(center);

            return nt;
        }
        public static Triangle MultiplXY(Triangle t1)
        {
            Point2f[] vectorDiffs = new Point2f[3];
            Point2f leftPoint = t1.left;
            Point2f center = t1.Centroid;

            for (int i = 0; i < 3; i++)
            {
                vectorDiffs[i] = t1.Vectors[i] * center.X;
            }

            Triangle nt = FromVectors(vectorDiffs, leftPoint);

            center = new Point2f(center.X, center.X * center.Y);

            nt.TranslateTo(center);

            return nt;
        }

        public static Triangle operator -(Triangle t1, TriangleDiff t2)
        {
            return t1 + (t2 * -1);
        }
    }
    static Point2f MidPoint2f(Point2f p1, Point2f p2)
    {
        return new Point2f((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
    }
    public void StopTracking()
    {
        isTracking = false;
        if (trackingThread != null && trackingThread.IsAlive)
        {
            trackingThread.Join();  // Wait for the thread to finish
        }
    }
    //public void TransmitData(Transform transform)
    //{
    //    // Set up the UDP client
    //    using (UdpClient udpClient = new UdpClient())
    //    {
    //        // Set the remote endPoint2f (IP address and port)
    //        IPEndPoint remoteEndPoint2f = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 9876); // Change IP if needed

    //        // Prepare the data as a comma-separated string of filtered values
    //        string data = transform.ToString();

    //        byte[] bytes = Encoding.UTF8.GetBytes(data);

    //        // Send the data
    //        udpClient.Send(bytes, bytes.Length, remoteEndPoint2f);
    //        //Console.WriteLine("Data transmitted: " + data + "\n");
    //    }
    //}
    public void ReleaseResources()
    {
        StopTracking();
        frame.Release();
        capture.Release();
    }
    public string GetTransformationDetails(float rotation, float scaleX, float scaleY, float shear, Point2f translation)
    {
        return $"Rotation (radians): {rotation}\n" +
               $"Scaling X: {scaleX}, Scaling Y: {scaleY}\n" +
               $"Shear: {shear}\n" +
               $"Translation: ({translation.X}, {translation.Y})";
    }
}
