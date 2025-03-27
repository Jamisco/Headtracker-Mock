using Headtracker_Console;
using OpenCvSharp;
using ScottPlot.Colormaps;
using ScottPlot.PlotStyles;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using static Headtracker_Console.Calibrator;

namespace Headtracker_Console
{
    public class HeadTracker
    {
        private VideoCapture capture;
        private Mat frame = new Mat();

        private Thread trackingThread;

        private Thread keyPressThread;

        public const int FRAMEWIDTH = 640;
        public const int FRAMEHEIGHT = 480;
        public static Point2f FRAMECENTER = new Point2f(FRAMEWIDTH / 2, FRAMEHEIGHT / 2);
        public const int CaptureFps = 30;
        public const bool TEST = false;
        public static bool SHOWGRAY = false;

        public Point2f printStartPos
        {
            get
            {
                Point2f p = new Point2f(10, 20);

                p = p + (printNewLine * printLine);
                printLine += 5;

                return p;
            }
        }
        public Point2f printNewLine = new Point2f(0, 20);
        int printLine = 1;

        public enum ShapeType { Triangle, Polygon, TShape };

        public const ShapeType Shape2Use = ShapeType.TShape;

        private bool isTracking;

        public enum Transformations { Scale, Yaw, Roll, Pitch, X, Y, Z };
        public enum Axis { X, Y };

        public KalmanFilter3D rotateKFilter;
        public KalmanFilter3D transKFilter;

        public SmoothingFilter rotSmoother;
        public SmoothingFilter traSmoother;

        float smoothValue = .2f;
        public HeadTracker(int cameraIndex = 0)
        {
            // Initialize the camera
            capture = new VideoCapture(cameraIndex);
            capture.Set(VideoCaptureProperties.Fps, CaptureFps); // Adjust as needed
            capture.FrameHeight = FRAMEHEIGHT;
            capture.FrameWidth = FRAMEWIDTH;
            Console.WriteLine("Size: " + capture.FrameHeight + " - " + capture.FrameWidth);
        }

        private void InitFilter()
        {
            rotateKFilter = new KalmanFilter3D();
            transKFilter = new KalmanFilter3D();
            rotSmoother = new SmoothingFilter(smoothValue);
            traSmoother = new SmoothingFilter(smoothValue);
        }

        // Detect contours or blobs (these would correspond to your LEDs/markers)
        public Triangle curTriangle { get; private set; }
        public Polygon curPolygon { get; private set; }
        public TShape curTShape { get; private set; }

        public Triangle centerTriangle { get; private set; }
        public Polygon centerPolygon { get; private set; }
        public TShape centerTShape { get; private set; }
        public bool HasCenter { get; set; } = false;

        public Calibrator calibrator = new Calibrator();

        public void StartTracking()
        {
            Console.WriteLine("Press a key to Begin...");

            Console.ReadKey();

            isTracking = true;
            trackingThread = new Thread(TrackingLoop);
            keyPressThread = new Thread(ReadKey);

            switch (Shape2Use)
            {
                case ShapeType.Triangle:
                    Console.WriteLine("Using Triangle Tracking");

                    break;
                case ShapeType.Polygon:
                    Console.WriteLine("Using Polygon Tracking");

                    break;
                case ShapeType.TShape:
                    Console.WriteLine("Using TShape Tracking");
                    break;
                default:
                    break;
            }

            trackingThread.Start();
            keyPressThread.Start();
        }

        public int frameCount = 0;

        public bool IsCalibrating { get; set; } = false;

        public void ReadKey()
        {
            while (true)
            {
                if (Console.KeyAvailable)
                {
                    ConsoleKeyInfo k = Console.ReadKey();

                    if (k.Key == ConsoleKey.Spacebar)
                    {
                        SetCenter();
                        SetOffset();

                        Console.WriteLine("New Center Set");
                    }
                    else if (k.Key == ConsoleKey.X)
                    {
                        Console.WriteLine("Prediction Cleared");
                        PoseTransformation.ClearOffsets();
                    }
                    else if (k.Key == ConsoleKey.A)
                    {
                        IsCalibrating = !IsCalibrating;

                        if(IsCalibrating)
                        {
                            calibrator.UpdateCenter(centerTShape, true);
                        }

                        string text = IsCalibrating ? "Started" : "Stopped";

                        Console.WriteLine(text + " Calibration");
                    }
                    else if (k.Key == ConsoleKey.Q)
                    {
                        if (IsCalibrating)
                        {
                            calibrator.CalibrateKeyPress(RotationType.Pitch);

                            string text = calibrator.CalibratingPitch ? "Started" : "Stopped";

                            Console.WriteLine(text + " Pitch Calibration");
                        }
                    }
                    else if (k.Key == ConsoleKey.W)
                    {
                        if (IsCalibrating)
                        {
                            calibrator.CalibrateKeyPress(RotationType.Yaw);

                            string text = calibrator.CalibratingYaw ? "Started" : "Stopped";

                            Console.WriteLine(text + " Yaw Calibration");
                        }
                    }
                    else if (k.Key == ConsoleKey.E)
                    {
                        if (IsCalibrating)
                        {
                            calibrator.CalibrateKeyPress(RotationType.Roll);

                            string text = calibrator.CalibratingRoll ? "Started" : "Stopped";

                            Console.WriteLine(text + " Roll Calibration");
                        }
                    }
                    else if (k.Key == ConsoleKey.S)
                    {
                        if (IsCalibrating)
                        {
                            calibrator.SaveCurrentData();

                        }
                    }
                    else if (k.Key == ConsoleKey.D)
                    {
                        if (IsCalibrating)
                        {
                            calibrator.ClearCurrentCalibration();

                        }
                    }
                }
            }
        }

        Mat displayFrame;
        public static Mat CloneFrame;

        string frameName = "Tracking Frame4";
        private void TrackingLoop()
        {
            Mat prevFrame = new Mat();
            // calibration frame completed, now we simply need to show the calibration points as we are calibrating or just skip straight to calculation
            while (isTracking)
            {
                capture.Read(frame);

                var ledPoints = ExtractLedPoints(frame);

                displayFrame = frame.EmptyClone();
                CloneFrame = frame.EmptyClone();
                DrawCenterGraph();

                switch (Shape2Use)
                {
                    case ShapeType.Triangle:
                        curTriangle = new Triangle(ledPoints);
                        curTriangle.ShowCurrentTriangle(displayFrame);

                        break;
                    case ShapeType.Polygon:

                        Polygon temp = new Polygon(ledPoints);

                        if (temp.IsValid)
                        {
                            curPolygon = temp;
                        }
                        curPolygon.ShowCurrentPolygon(displayFrame);

                        break;
                    case ShapeType.TShape:

                        TShape t = new TShape(ledPoints);

                        if (t.IsValid)
                        {
                            curTShape = t;
                        }

                        curTShape.ShowCurrentShape(displayFrame, printStartPos);

                        break;
                    default:
                        break;
                }
                printLine = 0;

                if (IsCalibrating)
                {
                    if (HasCenter)
                    {
                        calibrator.DrawGraph(displayFrame,  centerTShape.Centroid);

                        calibrator.DrawCenterData(displayFrame);

                        calibrator.AddShapeState(curTShape);

                    }
                    else
                    {
                        calibrator.DrawGraph(displayFrame,  curTShape.Centroid);
                    }
                }
                else
                {
                    if (HasCenter)
                    {
                        ShowCenterTriangle(displayFrame);
                    }

                    ShowHeadPose(displayFrame);
                }
                
InvalidPoint:
                ShowFrameCounter(displayFrame);

                Cv2.NamedWindow(frameName);
                Cv2.SetWindowProperty(frameName, WindowPropertyFlags.AspectRatio, 5);
                Cv2.ImShow(frameName, displayFrame);
                Cv2.WaitKey(1);

                frameCount++;
                prevFrame = frame.Clone();
            }
        }

        Point2f offset = new Point2f(0, 150);


        private List<Point2f> ExtractLedPoints(Mat frame)
        {
            try
            {
                Point[][] curContours;
                Mat grayFrame = new Mat();

                Cv2.CvtColor(frame, grayFrame, ColorConversionCodes.BGR2GRAY);

                HierarchyIndex[] hierarchy;

                // apply gausasain filter
                //Cv2.GaussianBlur(grayFrame, grayFrame, new Size(5, 5), 0);

                Cv2.Threshold(grayFrame, grayFrame, 30, 150, ThresholdTypes.Binary);


                if (SHOWGRAY)
                {
                    //Cv2.ImShow("OG Image", frame);

                    Cv2.ImShow("Gray Frame", grayFrame);
                }

                // pass in the previous points and modify the method so that it favors points that are closer to the previous points.
                // also make it so that points that are within the blob of said closer points are thesame as the previous points.
                Cv2.FindContours(grayFrame, out curContours, out hierarchy, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

                Point2f[] ledPoints;
                return PointFromContours(curContours);
            }
            catch (Exception ex)
            {
                return null;
            }
        }
        private List<Point2f> PointFromContours(Point[][] contours)
        {
            // Step 1: Calculate areas for all contours
            List<Tuple<Point[], double>> contourAreas = new List<Tuple<Point[], double>>();

            List<(Point2f, double)> pulses = new List<(Point2f, double)>();

            foreach (var contour in contours)
            {
                double area = Cv2.ContourArea(contour);
                contourAreas.Add(Tuple.Create(contour, area));
            }

            // Step 2: Sort the contours by area (descending order)
            contourAreas.Sort((a, b) => b.Item2.CompareTo(a.Item2));

            // Step 3: Take the largest contours 

            for (int i = 0; i < contourAreas.Count; i++)
            {
                double area = contourAreas[i].Item2;

                // there might be a situation where the light so dim such that the area is 0, even tho the light has been seen, in such a case, we will just use the position of the first contour because if the area is zero, that means whatever contours were found all have thesame position.
                if (area == 0)
                {
                    if (contourAreas[i].Item1.Length > 0)
                    {
                        pulses.Add((contourAreas[i].Item1[0], area));
                    }
                }
                else
                {
                    Moments moments = Cv2.Moments(contourAreas[i].Item1);

                    Point2f center = new Point2f((int)(moments.M10 / moments.M00), (int)(moments.M01 / moments.M00));

                    pulses.Add((center, area));
                }
            }

            // if 2 pulses are too close together remove the one with the smaller area

            float minDistance = 10;

            if (pulses.Count > 4)
            {
                int sds = 2;
            }

            for (int i = 0; i < pulses.Count; i++)
            {
                for (int j = i + 1; j < pulses.Count; j++)
                {
                    if (pulses[i].Item1.DistanceTo(pulses[j].Item1) < minDistance)
                    {
                        if (pulses[i].Item2 > pulses[j].Item2)
                        {
                            pulses.RemoveAt(j);
                        }
                        else
                        {
                            pulses.RemoveAt(i);
                        }
                    }
                }
            }

            for (int i = 0; i < pulses.Count; i++)
            {
                Point2f p = pulses[i].Item1;
                double area = pulses[i].Item2;

                // Mirror across the center of the frame
                p.X = FRAMEWIDTH - p.X;  // This flips relative to frame width

                pulses[i] = (p, area);
            }

            return pulses.Select(p => p.Item1).ToList();
        }
        private void ShowCenterTriangle(Mat displayFrame)
        {
            Mat f = frame.EmptyClone();

            if (HasCenter)
            {
                switch (Shape2Use)
                {
                    case ShapeType.Triangle:

                        centerTriangle.DrawShape(displayFrame, Scalar.White, true);
                        centerTriangle.DrawCentriod(displayFrame);

                        break;
                    case ShapeType.Polygon:

                        centerPolygon.DrawShape(displayFrame, Scalar.White, true);
                        centerPolygon.DrawCentriod(displayFrame);

                        break;
                    case ShapeType.TShape:

                        centerTShape.DrawShape(displayFrame, Scalar.White, true);
                        centerTShape.DrawCentriod(displayFrame);

                        centerTShape.PrintData(displayFrame, printStartPos);

                        break;
                    default:
                        break;
                }
            }
        }
        private void ShowFrameCounter(Mat displayFrame)
        {
            Point bottom = new Point(0, 580);

            Cv2.PutText(displayFrame, "Frame Count: " + frameCount, bottom, HersheyFonts.HersheyPlain, 1, Scalar.White);
        }
        private void SetCenter()
        {
            switch (Shape2Use)
            {
                case ShapeType.Triangle:

                    centerTriangle = curTriangle;

                    break;
                case ShapeType.Polygon:

                    centerPolygon = curPolygon;

                    break;
                case ShapeType.TShape:

                    centerTShape = curTShape;

                    break;
                default:
                    break;
            }

            HasCenter = true;
            InitFilter();
        }
        private void DrawCenterGraph()
        {
            Point2f topMid, botMid;
            Point2f leftMid, rightMid;

            Size frameSize = displayFrame.Size();

            float height = frameSize.Height;
            float width = frameSize.Width;

            topMid = new Point2f(width / 2, 0);
            botMid = new Point2f(width / 2, height);

            Point2f yAdjust = new Point2f(0, 15);

            leftMid = new Point2f(0, height / 2) - yAdjust;
            rightMid = new Point2f(width, height / 2) - yAdjust;

            Scalar col = Scalar.Teal;
            int s = 1;

            Cv2.Line(displayFrame, topMid.R2P(), botMid.R2P(), col, s);
            Cv2.Line(displayFrame, leftMid.R2P(), rightMid.R2P(), col, s);
        }
        private void SetOffset()
        {
            switch (Shape2Use)
            {
                case ShapeType.Triangle:

                    PoseTransformation.SetCenter(curTriangle.Points);

                    break;
                case ShapeType.Polygon:

                    PoseTransformation.SetCenter(curPolygon.Points);

                    break;
                case ShapeType.TShape:

                    PoseTransformation.SetCenter(curTShape.Points);

                    break;
                default:
                    break;
            }
        }

        int maxD = -1;

        private void ShowHeadPose(Mat displayFrame)
        {
            Point3d r, t;

            Point start = new Point(0, 300);
            Point step = new Point(0, 20);
            int c = 0;

            Point2f[] points;

            switch (Shape2Use)
            {
                case ShapeType.Triangle:
                    points = curTriangle.Points;
                    break;
                case ShapeType.Polygon:
                    points = curPolygon.Points;
                    break;
                case ShapeType.TShape:
                    points = curTShape.Points;

                    break;
                default:
                    break;
            }

            try
            {
                //PoseTransformation.EstimateTransformation(points, out Point3d r2, out Point3d t2);

                //PoseTransformation.EstimateTransformation2(displayFrame, centerTShape, curTShape, out Point3f r2, out Point3f t2);

                // save unit vectors so we dont have to keep calibrating.
                // consider finding that rotation origin instead of using unit vectors.

                if (!calibrator.HasCalibration)
                {
                    return;
                }

                // some values dont really cross the center of the frame, thus the postive and negative values are not really accurate or sometimes no existent. New methods must be devised to store states

                //calibrator.GetUnitVector(RotationType.Pitch, out UnitVector puv);
                //calibrator.GetUnitVector(RotationType.Yaw, out UnitVector yuv);
                //calibrator.GetUnitVector(RotationType.Roll, out UnitVector ruv);


                calibrator.GetOrigin(RotationType.Roll, out Point2f origin);

                //PoseTransformation.EstimateTransformation3(displayFrame, centerTShape, curTShape, puv, yuv, ruv, out Point3f r2, out Point3f t2);

                PoseTransformation.EstimateTransformation5(displayFrame, curTShape,  out Point3f r2, out Point3f t2);

                //Cv2.PutText(displayFrame, "Rotation: " + r.R2P(), 
                //    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                //Cv2.PutText(displayFrame, "Translation: " + t.R2P(), 
                //    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                c += 2;

                deadzone = .5f;
                r2 = EngageDeadzone(prevR, r2);
                //t2 = EngageDeadzone(prevT, t2);
                float sv = .4f;


                // the problem is the random up and down jitter
                // you cant smooth these out using filter because they look like genuine movement.
                // even as the head moves, this jitter is there, albeit less noticeable.
                rotSmoother.ChangeAlpha(sv);
                traSmoother.ChangeAlpha(sv);

                r2 = rotSmoother.Smooth(r2);
                r2 = rotateKFilter.Update(r2);

                t2 = traSmoother.Smooth(t2);
                t2 = transKFilter.Update(t2);

                Cv2.PutText(displayFrame, "Rotation2: " + r2.R2P(2),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                Cv2.PutText(displayFrame, "Translation2: " + t2.R2P(),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                prevR = r2;
                prevT = t2;

                //SendData(r2, t2);
                SendData2OpenTrack(r2, t2);

                //if (HasCenter)
                //{
                //    SendData(r2);
                //}   
            }
            catch (Exception ex)
            {

                return;
            }
        }

        float deadzone = 1f;

        private Point3f EngageDeadzone(Point3f prev, Point3f cur)
        {
            if (Math.Abs(cur.X - prev.X) < deadzone)
            {
                cur.X = prev.X;
            }

            if (Math.Abs(cur.Y - prev.Y) < deadzone)
            {
                cur.Y = prev.Y;
            }

            if (Math.Abs(cur.Z - prev.Z) < deadzone)
            {
                cur.Z = prev.Z;
            }

            return cur;
        }

        private Point3f prevR = new Point3f();
        private Point3f prevT = new Point3f();

        public void DrawCircleAtPoint(Point2f point2Draw, Scalar sl)
        {
            Cv2.Circle(displayFrame, point2Draw.R2P(), 2, sl, 5);
        }

        private static UdpClient udpClient;
        private static readonly string localhost = "127.0.0.1";
        private static readonly int port = 4242;
        public static void SendData(Point3f r, Point3f t)
        {
            string pitch = r.X.ToString("F2");
            string yaw = r.Y.ToString("F2");
            string roll = r.Z.ToString("F2");

            string x = t.X.ToString("F2");
            string y = t.Y.ToString("F2");
            string z = t.Z.ToString("F2");

            if (udpClient == null)
            {
                udpClient = new UdpClient();
            }

            try
            {
                // Format: "pitch,yaw,roll,tx,ty,tz"
                string data = $"{pitch},{yaw},{roll},{x},{y},{z}";
                byte[] bytes = Encoding.UTF8.GetBytes(data);
                udpClient.Send(bytes, bytes.Length, localhost, port);
            }
            catch (Exception e)
            {
                Console.WriteLine($"UDP Send Error: {e.Message}");
            }
        }

        public static void SendData2OpenTrack(Point3f r, Point3f t)
        {
            using (UdpClient client = new UdpClient())
            {
                client.Connect(localhost, port);

                // Use double (not float) since OpenTrack requires 64-bit values
                double[] data = { t.X, t.Y, t.Z, r.Y, -r.X, r.Z };
                byte[] bytes = new byte[data.Length * sizeof(double)]; // Ensure correct size: 6 * 8 bytes = 48 bytes

                Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length); // Copy the data correctly

                client.Send(bytes, bytes.Length); // Send the UDP packet

            }
        }

        public void StopTracking()
        {
            isTracking = false;
            if (trackingThread != null && trackingThread.IsAlive)
            {
                trackingThread.Join();  // Wait for the thread to finish
            }
        }
        public void ReleaseResources()
        {
            StopTracking();
            frame.Release();
            capture.Release();
        }
    }
}