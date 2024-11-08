using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading;
using Headtracker_Console;
using Emgu.CV.Structure;
using System.Numerics;
using static HeadTracker;


class HeadTracker
{
    private VideoCapture capture;
    private Mat frame = new Mat();
    private Thread trackingThread;
    private Thread calibrateThread;

    private bool isTracking;

    public HeadTracker(int cameraIndex = 0)
    {
        // Initialize the camera
        capture = new VideoCapture(cameraIndex);
        capture.Set(VideoCaptureProperties.Fps, 30); // Adjust as needed
        capture.FrameHeight = 800;
        capture.FrameWidth = 800;
        Console.WriteLine("Size: " + capture.FrameHeight + " - " + capture.FrameWidth);
    }

    public void StartTracking()
    {
        isTracking = true;
        trackingThread = new Thread(TrackingLoop);
        calibrateThread = new Thread(BeginCalibration);

        trackingThread.Start();
        calibrateThread.Start();
    }

    int frameCount = 0;
    private void TrackingLoop()
    {
        while (isTracking)
        {
            capture.Read(frame);

            if (frame.Empty())
            {
                continue;
            }

            ProcessFrame(frame);

            frameCount++;
            // Process the frame to detect markers (LEDs)
            //ProcessFrame2(frame);

            //// Show the frame with markers for debugging
            //   Cv2.ImShow("Tracking", frame);
            Cv2.WaitKey(1);
        }
    }

    // Detect contours or blobs (these would correspond to your LEDs/markers)
    Point[][] curContours;
    Point[][] prevContours;

    Triangle curTri = new Triangle();
    Triangle prevTri = new Triangle();

    Transform data = new Transform();
    Calibration pitchCal = new Calibration();

    public bool begin = false;

    int t = 20;

    Point2f start = new Point2f(10, 20);
    Point2f offset = new Point2f(0, 150);
    private void ProcessFrame(Mat frame)
    {
        Mat grayFrame = new Mat();

        Cv2.CvtColor(frame, grayFrame, ColorConversionCodes.BGR2GRAY);

        HierarchyIndex[] hierarchy;

        // apply gausasain filter
        Cv2.GaussianBlur(grayFrame, grayFrame, new Size(5, 5), 0);

        Cv2.Threshold(grayFrame, grayFrame, 100, 255, ThresholdTypes.Binary);

        Cv2.FindContours(grayFrame, out curContours, out hierarchy, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

        GetLedContours(ref curContours);

        Triangle temp = curTri;

        try
        {
            GetTrianglePoints(curContours, out curTri);

            prevTri = temp;

            Mat f = grayFrame.EmptyClone();
            Cv2.CvtColor(f, f, ColorConversionCodes.GRAY2BGR);

            curTri.DrawTriangle(f, Scalar.Red);

            centeredTriangle.DrawTriangle(f, Scalar.White, false);
            centeredTriangle.DrawCentriod(f);
            centeredTriangle.PrintData(f, start);

            if (calibrated)
            {
                int i = 1;

                Point2f p;
                if (pitchCal.Calibrate(curTri, out Triangle offTri, out p))
                {
                    // Cv2.Circle(f, p, 5, Scalar.White, 2);
                    offTri.DrawTriangle(f, Scalar.Yellow);

                    // find a way to get the side lengths of a triangle
                    // your gonna have to track the Point2fs of the triangle
                    // or perhaps rescale the vectors
                    // keep track the of angle of each side, this way we can easily determine the side lengths
                }

                Cv2.ImShow("Gray Frame 2", f);
            }
        }
        catch (Exception)
        {


        }

        //Cv2.ImShow("Gray Frame", grayFrame);
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

    public bool calibrated = false;
    public void BeginCalibration()
    {
        // index 0 = top Point2f
        // index 1 = left Point2f
        // index 2 = right Point2f

        Console.WriteLine("Press a key to Begin Calibration...");

        bool cal = false;
        List<Triangle> points = new List<Triangle>();

        int lastFrameCount = 0;

        while (true)
        {
            if (Console.KeyAvailable)
            {
                ConsoleKeyInfo k = Console.ReadKey();

                if (k.Key == ConsoleKey.Spacebar)
                {
                    CalibrateCenter();
                    Console.WriteLine("Calibrated Center");
                }
                else if (k.Key == ConsoleKey.Q)
                {
                    if (cal == false)
                    {
                        cal = true;
                        Console.WriteLine("Pitch Calibration Started");
                    }
                    else
                    {
                        cal = false;
                        Console.WriteLine("Pitch Calibration Stopped");
                        pitchCal = new Calibration(points, centeredTriangle, Axis.YAxis);

                        //CalibratePitch();
                    }

                }
                else if (k.Key == ConsoleKey.C)
                {
                    ClearCalibrations();
                    points.Clear();
                    Console.WriteLine("Cleared Calibration");
                }
            }

            if (cal == true && frameCount > lastFrameCount)
            {
                bool GoodRange(Point2f num)
                {
                    if (num.X >= 0 && num.X <= frame.Width)
                    {
                        if (num.Y >= 0 && num.Y <= frame.Height)
                        {
                            return true;
                        }
                    }

                    return false;
                }

                if (GoodRange(curTri.centroid))
                {
                    points.Add(curTri);
                    lastFrameCount = frameCount;
                }
            }

        }
    }

    Triangle centeredTriangle = new Triangle();
    public void CalibrateCenter()
    {
        centeredTriangle = curTri;

        if (calibrated)
        {
            pitchCal.UpdateCenter(centeredTriangle);
        }

        calibrated = true;
    }

    public List<Triangle> triangles = new List<Triangle>();
    List<Point2f> points = new List<Point2f>();
    public void CalibratePitch()
    {
        //points.Add(curTri.centroid);
        //tOffset = new Toffset(centeredTriangle, curTri);
    }

    public void ClearCalibrations()
    {
        //calibrate.Calibrate(centeredTriangle, curTri);
        triangles.Clear();
    }

    KalmanFilterFloat kf = new KalmanFilterFloat();

    static float maxRotation = 45;
    static float maxTranslation = 100;

    public enum Axis { XAxis, YAxis, All };


    public enum Quadrant { First, Second, Third, Fourth, All };

    public struct Toffset
    {
        /// <summary>
        /// The max value of a transformation
        /// </summary>

        Triangle centerTriangle;


        Point2f unitMultiplier;

        Axis axis;

        float[] offsetRatio;
        float[] offsetDiff;
        bool xFactor;

        static float tolerance = .05f;

        public static Quadrant GetQuadrant(Triangle center, Triangle other)
        {
            Quadrant quadrant;

            Point2f axisOffset = other.centroid - center.centroid;

            if (axisOffset.X > 0 && axisOffset.Y > 0)
            {
                quadrant = Quadrant.First;
            }
            else if (axisOffset.X < 0 && axisOffset.Y > 0)
            {
                quadrant = Quadrant.Second;
            }
            else if (axisOffset.X < 0 && axisOffset.Y < 0)
            {
                quadrant = Quadrant.Third;
            }
            else
            {
                quadrant = Quadrant.Fourth;
            }

            return quadrant;
        }
        public Triangle ScaleTriangle(float[] offsetRatio, Triangle tri, Point2f c)
        {
            // o is the original triangle with 3 Point2fs: o[0] = top, o[1] = right, o[2] = left
            // c is the center Point2f
            // offsetRatio contains the scaling ratios for each vertex

            // Create a new array to hold the new scaled Point2fs
            Point2f[] scaledTriangle = new Point2f[3];

            List<Point2f> o = tri.Points;

            // Iterate over each vertex of the triangle (top, right, left)
            for (int i = 0; i < 3; i++)
            {
                // Calculate the vector from center 'c' to the triangle vertex 'o[i]'
                Point2f vector = new Point2f(o[i].X - c.X, o[i].Y - c.Y);

                // Scale the vector by the corresponding scaling factor from the offsetRatio array
                Point2f scaledVector = new Point2f(vector.X * offsetRatio[i], vector.Y * offsetRatio[i]);

                // Find the new position of the vertex by adding the scaled vector to the center 'c'
                scaledTriangle[i] = new Point2f(c.X + scaledVector.X, c.Y + scaledVector.Y);
            }

            return new Triangle(scaledTriangle.ToList()); // Return the new scaled triangle vertices
        }

        bool WithinTolerance(float[] t1, float[] t2)
        {
            for (int i = 0; i < 3; i++)
            {
                float n = Math.Abs(t1[i] - t2[i]);

                if (n > tolerance)
                {
                    return false;
                }
            }

            return true;
        }
    }

    public struct Calibration
    {
        public Triangle centerTriangle;

        public Triangle maxTriangle;
        public Triangle minTriangle;

        public Point2f maxPointOffset;
        public Point2f minPointOffset;

        // multipliers are used to account for deviation across the invert axis transformations
        // This is because the head doesnt move in a straight line. So a pitch movement, will also have a yaw movement, we there need to account for this. Subsequently, we need to account for the deviation in both sides of the axis. Someones head might have deviation in the postive pitch axis, but not in the negative pitch axis. This is why we need to account for both sides of the axis
        public Point2f minMultiplier;
        public Point2f maxMultiplier;

        float[] minSideDiff;
        float[] maxSideDiff;

        public Axis axis;

        public Calibration(List<Triangle> points, Triangle center, Axis axis)
        {
            this.axis = axis;
            this.centerTriangle = center;

            if (axis == Axis.XAxis)
            {
                points.Sort((a, b) => a.centroid.X.CompareTo(b.centroid.X));

                minTriangle = points[0];
                maxTriangle = points[points.Count - 1];

                maxPointOffset = (maxTriangle.centroid - center.centroid).Abs();
                minPointOffset = (minTriangle.centroid - center.centroid).Abs();

                minSideDiff = Triangle.Difference(center, minTriangle).sides;
                maxSideDiff = Triangle.Difference(center, maxTriangle).sides;

                if (minPointOffset.X == 0)
                {
                    minMultiplier = new Point2f(0, 1);
                }
                else
                {
                    minMultiplier = new Point2f(1, minPointOffset.Y / minPointOffset.X);
                }

                if (maxPointOffset.X == 0)
                {
                    maxMultiplier = new Point2f(0, 1);
                }
                else
                {
                    maxMultiplier = new Point2f(1, maxPointOffset.Y / maxPointOffset.X);
                }
            }
            else
            {
                points.Sort((a, b) => a.centroid.Y.CompareTo(b.centroid.Y));

                minTriangle = points[0];
                maxTriangle = points[points.Count - 1];

                maxPointOffset = (maxTriangle.centroid - center.centroid).Abs();
                minPointOffset = (minTriangle.centroid - center.centroid).Abs();

                minSideDiff = Triangle.Difference(center, minTriangle).sides;
                maxSideDiff = Triangle.Difference(center, maxTriangle).sides;

                if (minPointOffset.Y == 0)
                {
                    minMultiplier = new Point2f(1, 0);
                }
                else
                {
                    minMultiplier = new Point2f(minPointOffset.X / minPointOffset.Y, 1);
                }

                if (maxPointOffset.Y == 0)
                {
                    maxMultiplier = new Point2f(1, 0);
                }
                else
                {
                    maxMultiplier = new Point2f(maxPointOffset.X / maxPointOffset.Y, 1);
                }
            }

        }
        private Point2f OffsetPoint(Point2f cp, out float percent, out float[] sideDiff)
        {
            Point2f offset = new Point2f();

            int yNeg = 1;
            int xNeg = 1;

            float yOffsetLimit = 0;
            float xOffsetLimit = 0;

            Point2f multiplier;

            // Essentially, what we are saying here is that if the point is above the center, then we want to use offset by reducing the y value. If the point is to the right of the center, then we want to offset reduce the x value. Hence the -1
            yNeg = cp.Y > centerTriangle.centroid.Y ? -1 : 1;
            xNeg = cp.X > centerTriangle.centroid.X ? -1 : 1;
            
            if (axis == Axis.YAxis)
            {
                if (yNeg == -1)
                {
                    // below center
                    multiplier = maxMultiplier;
                    yOffsetLimit = maxPointOffset.Y;
                    xOffsetLimit = maxPointOffset.X;
                    sideDiff = maxSideDiff;
                }
                else
                {
                    // above center
                    multiplier = minMultiplier;
                    yOffsetLimit = minPointOffset.Y;
                    xOffsetLimit = minPointOffset.X;
                    sideDiff = minSideDiff;
                }

                offset.Y = Math.Abs((centerTriangle.centroid.Y - cp.Y));

                if (offset.Y > yOffsetLimit)
                {
                    offset.Y = yOffsetLimit;
                }

                offset.X = (multiplier.X * offset.Y);
            }
            else
            {
                if(xNeg == -1)
                {
                    // right of center
                    multiplier = maxMultiplier;
                    xOffsetLimit = maxPointOffset.X;
                    yOffsetLimit = maxMultiplier.Y;
                    sideDiff = maxSideDiff;
                }
                else
                {
                    // left of center
                    multiplier = minMultiplier;
                    xOffsetLimit = minMultiplier.X;
                    yOffsetLimit = minPointOffset.Y;
                    sideDiff = minSideDiff;
                }

                offset.X = Math.Abs((centerTriangle.centroid.X - cp.X));

                if (offset.X > xOffsetLimit)
                {
                    offset.X = xOffsetLimit;
                }

                offset.Y = (multiplier.Y * offset.X);
            }

            offset = new Point2f(offset.X * xNeg, offset.Y * yNeg);

            try
            {
                if (axis == Axis.XAxis)
                {
                    percent = offset.X * 1f / xOffsetLimit;
                }
                else
                {
                    percent = offset.Y * 1f / yOffsetLimit;
                }

                percent = Math.Abs(percent);
            }
            catch (Exception)
            {
                percent = 1;
            }

            return offset;
        }
        public void UpdateCenter(Triangle center)
        {
            this.centerTriangle = center;

            minSideDiff = Triangle.Difference(center, minTriangle).sides;
            maxSideDiff = Triangle.Difference(center, maxTriangle).sides;
        }
        public bool Calibrate(Triangle curTriangle, out Triangle offsetTriangle, out Point2f np)
        {
            float[] diff = curTriangle.SideLengths;
            float percent = 1;

            float[] sideDiff;
            Point2f offset = OffsetPoint(curTriangle.centroid, out percent, out sideDiff);

            float[] newSides = new float[3];

            try
            {
                if (axis == Axis.XAxis)
                {
                    // use x to determine the percent of the offset
                }
                else
                {
                    // use y to determine the percent of the offset
                    for (int i = 0; i < 3; i++)
                    {
                        newSides[i] = (diff[i] + (sideDiff[i] * percent)) / diff[i];
                    }
                }
            }
            catch (Exception)
            {
            }

            np = curTriangle.centroid + offset;

            // the base length isn't accurate
            curTriangle.ScaleVectors(newSides);
            curTriangle.TranslateTo(np);

            offsetTriangle = curTriangle;

            return true;
        }
    }

    public struct Transform
    {
        public float Yaw { get; set; }   // Horizontal rotation in degrees
        public float Pitch;  // Vertical rotation in degrees
        public float Roll;   // Tilt rotation in degrees
        public float X;      // X position
        public float Y;      // Y position
        public float Z;      // Z position

        Triangle centerTriangle;
        Calibration pc;

        public Transform(Calibration pitchCal)
        {
            Yaw = 0;
            Pitch = 0;
            Roll = 0;
            X = 0;
            Y = 0;
            Z = 0;

            pc = pitchCal;
            centerTriangle = pitchCal.centerTriangle;
        }

        /// <summary>
        /// Scale Tolerance
        /// </summary>
        static float st = .05f;
        public void GetTransformation(ref Triangle t2, out Triangle transformed)
        {
            TransformPitch(ref t2, out transformed);
        }
        private void TransformScale(ref Triangle t1, ref Triangle t2)
        {
            float tl = t2.Top2Left / t1.Top2Left;
            float tr = t2.Top2Right / t1.Top2Right;
            float lr = t2.Left2Right / t1.Left2Right;

            int neg = t2.top.Y > t1.top.Y ? -1 : 1;

            Z = (tl + tr + lr) / 3;

            t2.Scale(Z);

            Z = (1 - Z) * neg;
        }
        private void TransformPitch(ref Triangle t1, out Triangle transformed)
        {
            pc.Calibrate(t1, out transformed, out Point2f np);
        }
        private void TransformRoll(ref Triangle t1, ref Triangle t2)
        {
            // we determine the roll, we measure the difference in height between left and right

            float hDiff1 = t1.left.Y - t1.right.Y;
            float hDiff2 = t2.left.Y - t2.right.Y;

            Roll = (hDiff2 - hDiff1) / t1.Left2Right;

            t2.Rollback(t1);

            // t2.ro
        }
        private void TransformYaw(ref Triangle t1, ref Triangle t2)
        {
            // we determine the yaw by checking the left to right
            // if the left to right is reduced, then its a roll

            float lr = Math.Abs(1 - (t2.Left2Right / t1.Left2Right));
            int neg = t2.top.X > t1.top.X ? -1 : 1;

            Yaw = lr;

            if (Yaw >= .95f)
            {
                int seds = 4;
                Yaw = 0;
            }

            t2.Yaw(Yaw);

            Yaw = Yaw * neg;
        }
        private void TransformTranslate(ref Triangle t1, ref Triangle t2)
        {
            Point2f p = t2.centroid - t1.centroid;

            X = -p.X;
            Y = -p.Y;

            t2.Translate(p);
        }

        private void ValidateValues()
        {
            // if any properties are NAN, set to 0

            if (float.IsNaN(Yaw))
            {
                Yaw = 0;
            }

            if (float.IsNaN(Pitch))
            {
                Pitch = 0;
            }

            if (float.IsNaN(Roll))
            {
                Roll = 0;
            }

            if (float.IsNaN(X))
            {
                X = 0;
            }

            if (float.IsNaN(Y))
            {
                Y = 0;
            }

            if (float.IsNaN(Z))
            {
                Z = 0;
            }
        }
        public override string ToString()
        {
            ValidateValues();

            string str = $"{Yaw},{Pitch},{Roll},{X},{Y},{Z}";

            return str;
        }

        public void Print()
        {
            ValidateValues();

            Console.WriteLine("Yaw: " + Yaw.ToString("F3") +
                              "\tPitch: " + Pitch.ToString("F3") +
                              "\tRoll: " + Roll.ToString("F3") +
                              "\tX: " + X.ToString("F3") +
                              "\tY: " + Y.ToString("F3") +
                              "\tZ: " + Z.ToString("F3"));
        }
    }

    public struct Triangle
    {
        public Point2f top;
        public Point2f left;
        public Point2f right;

        public Point2f centroid;

        public float Top2Left;
        public float Top2Right;
        public float Left2Right;

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

        public List<Point2f> Points
        {
            get
            {
                List<Point2f> Point2fs = new List<Point2f>();

                Point2fs.Add(top);
                Point2fs.Add(right);
                Point2fs.Add(left);

                return Point2fs;
            }
        }
        public List<Point2f> Vectors
        {
            get
            {
                List<Point2f> vectors = new List<Point2f>();

                // left to top
                vectors.Add(top - left);

                // top to right
                vectors.Add(right - top);

                // right to left
                vectors.Add(left - right);

                return vectors;
            }
        }
        public Triangle(List<Point2f> cur)
        {
            this.top = cur[0];
            this.right = cur[1];
            this.left = cur[2];

            Top2Left = 0;
            Top2Right = 0;
            Left2Right = 0;

            centroid = new Point2f();

            ValidateDimensions();
        }

        public void ScalePoints(float[] sides)
        {
            Point2f center = centroid;

            for (int i = 0; i < 3; i++)
            {
                Point2f p = Points[i];

                p = p * sides[i];

                switch (i)
                {
                    case 0:
                        top = p;

                        break;
                    case 1:
                        right = p;
                        break;
                    case 2:
                        left = p;
                        break;
                }
            }

            ValidateDimensions();
        }
        public void ScaleVectors(float[] sides)
        {
            List<Point2f> vectors = Vectors;

            Point2f center = centroid;

            for (int i = 0; i < 3; i++)
            {
                Point2f v = vectors[i];

                v = v * sides[i];

                switch (i)
                {
                    case 0:
                        top = left + v;

                        break;
                    case 1:
                        right = top + v;
                        break;
                    case 2:
                        //left = new Point2f(right.X + v.X, right.Y + v.Y);

                        break;
                }

            }

            ValidateDimensions();
        }

        private static Point2f GetVector(Point2f a, Point2f b)
        {
            return new Point2f(b.X - a.X, b.Y - a.Y);
        }
        public Triangle(Mat matrix)
        {
            // Extract Point2fs from the matrix
            top = new Point2f(matrix.At<float>(0, 0), matrix.At<float>(0, 1));
            right = new Point2f(matrix.At<float>(1, 0), matrix.At<float>(1, 1));
            left = new Point2f(matrix.At<float>(2, 0), matrix.At<float>(2, 1));

            Top2Left = 0;
            Top2Right = 0;
            Left2Right = 0;

            centroid = new Point2f();

            ValidateDimensions();
        }

        public Mat Coordinates
        {
            get
            {
                Mat c = new Mat(3, 3, MatType.CV_32F);

                // Set the rows using top, right, and left in homogeneous coordinates
                c.Set<float>(0, 0, top.X); // X coordinate of the top Point2f
                c.Set<float>(0, 1, top.Y); // Y coordinate of the top Point2f
                c.Set<float>(0, 2, 1.0f);    // Homogeneous coordinate for 'top'

                c.Set<float>(1, 0, right.X); // X coordinate of the right Point2f
                c.Set<float>(1, 1, right.Y); // Y coordinate of the right Point2f
                c.Set<float>(1, 2, 1.0f);      // Homogeneous coordinate for 'right'

                c.Set<float>(2, 0, left.X); // X coordinate of the left Point2f
                c.Set<float>(2, 1, left.Y); // Y coordinate of the left Point2f
                c.Set<float>(2, 2, 1.0f);     // Homogeneous coordinate for 'left'

                return c;
            }
        }

        private void ValidateDimensions()
        {
            Top2Left = (float)Point2f.Distance(top, left);
            Top2Right = (float)Point2f.Distance(top, right);
            Left2Right = (float)Point2f.Distance(left, right);

            centroid = new Point2f((top.X + left.X + right.X) / 3, (top.Y + left.Y + right.Y) / 3);
        }

        //public Triangle(Point2f centriod, float[] sides)
        //{

        //}

        public static (float scale, float yaw, float roll, float pitch, float distance) Compare(Triangle t1, Triangle t2)
        {
            float pitch = 1 - (t2.Top2Left + t2.Top2Right) / (t1.Top2Left + t1.Top2Right);

            float scale = 1 - (t2.Perimeter / t1.Perimeter);

            float yaw = 1 - (t2.Left2Right / t1.Left2Right);

            float hDiff1 = t1.left.Y - t1.right.Y;
            float hDiff2 = t2.left.Y - t2.right.Y;

            float roll = ((hDiff2 - hDiff1) / t1.Left2Right);

            float distance = (float)Point2f.Distance(t1.centroid, t2.centroid);

            scale *= 100;
            yaw *= 100;
            roll *= 100;
            pitch *= 100;

            return (scale, yaw, roll, pitch, distance);

        }

        /// <summary>
        /// Will return the differences between t2 and t1. Will do t2 - t1
        /// </summary>
        /// <param name="t1"></param>
        /// <param name="t2"></param>
        /// <returns></returns>
        public static (float[] sides, Point2f c) Difference(Triangle t1, Triangle t2)
        {
            (float[] side, Point2f c) diff;

            diff.side = new float[3];

            diff.side[0] = t1.Top2Left - t2.Top2Left;
            diff.side[1] = t1.Top2Right - t2.Top2Right;
            diff.side[2] = t1.Left2Right - t2.Left2Right;
            diff.c = t1.centroid - t2.centroid;

            return diff;

        }

        public float[] GetRatios()
        {
            float[] r = new float[3];

            // can be used to identify orientation too
            // if triangle ever gets flipped
            float p = Perimeter;

            r[0] = Top2Left / p;
            r[1] = Top2Right / p;
            r[2] = Left2Right / p;

            return r;
        }
        public void Scale(float scaleFactor)
        {
            top = ((top - centroid) * scaleFactor) + centroid;
            right = ((right - centroid) * scaleFactor) + centroid;
            left = ((left - centroid) * scaleFactor) + centroid;

            ValidateDimensions();
        }
        public void Pitch(float pitchFactor)
        {
            top = new Point2f(top.X, top.Y * pitchFactor);

            ValidateDimensions();
        }

        public void Yaw(float yawFactor)
        {
            left = new Point2f(left.X * yawFactor, left.Y);
            right = new Point2f(right.X * yawFactor, right.Y);

            ValidateDimensions();
        }

        public void Rollback(Triangle t1)
        {
            Point2f cDiff = centroid - t1.centroid;

            top += cDiff;
            left += cDiff;
            right += cDiff;

            ValidateDimensions();
        }
        public void Translate(Point2f p)
        {
            top -= p;
            left -= p;
            right -= p;

            ValidateDimensions();
        }

        public void TranslateTo(Point2f center)
        {
            // move current triangle to new center

            Point2f cDiff = center - centroid;

            top += cDiff;
            left += cDiff;
            right += cDiff;

            ValidateDimensions();
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
            Cv2.Circle(frame, centroid.R2P(), 2, sl, 2);

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
            string top1 = "(" + top.X + ", " + top.Y + ")";
            string right1 = "(" + right.X + ", " + right.Y + ")";
            string left1 = "(" + left.X + ", " + left.Y + ")";

            Cv2.PutText(frame, top1, (top - py).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, right1, (right + px).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, left1, ((left - xp + py).R2P()), HersheyFonts.HersheyPlain, 1, sl);
        }
        public void DrawCentriod(Mat frame)
        {
            Cv2.Circle(frame, centroid.R2P(), 2, Scalar.White, 2);
        }
        public void PrintData(Mat frame, Point2f pos)
        {
            // print the center location and the lengths

            Point2f px = new Point2f(20, 0);
            Point2f py = new Point2f(0, 20);

            Scalar sl = Scalar.White;

            int c = 0;

            Cv2.PutText(frame, "Centroid: " + centroid.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Top: " + top.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Right: " + right.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Left: " + left.ToString2(), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Top2Left: " + Top2Left.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            Cv2.PutText(frame, "Top2Right: " + Top2Right.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            Cv2.PutText(frame, "Left2Right: " + Left2Right.ToString("F2"), (pos + (py * c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
        }
        private Point2f GetTranslation(Triangle o, Triangle m)
        {
            Point2f translation = new Point2f();

            translation.X = m.centroid.X - o.centroid.X;
            translation.Y = m.centroid.Y - o.centroid.Y;

            return translation;
        }

    }

    static Point2f MidPoint2f(Point2f p1, Point2f p2)
    {
        return new Point2f((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
    }

    double RTD(double r)
    {
        return r * 180 / Math.PI;
    }

    public void StopTracking()
    {
        isTracking = false;
        if (trackingThread != null && trackingThread.IsAlive)
        {
            trackingThread.Join();  // Wait for the thread to finish
        }
    }
    public void TransmitData(Transform transform)
    {
        // Set up the UDP client
        using (UdpClient udpClient = new UdpClient())
        {
            // Set the remote endPoint2f (IP address and port)
            IPEndPoint remoteEndPoint2f = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 9876); // Change IP if needed

            // Prepare the data as a comma-separated string of filtered values
            string data = transform.ToString();

            byte[] bytes = Encoding.UTF8.GetBytes(data);

            // Send the data
            udpClient.Send(bytes, bytes.Length, remoteEndPoint2f);
            //Console.WriteLine("Data transmitted: " + data + "\n");
        }
    }
    public void ReleaseResources()
    {
        StopTracking();
        frame.Release();
        capture.Release();
    }
}
