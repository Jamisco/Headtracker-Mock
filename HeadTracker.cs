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

        public enum ShapeType { Triangle, Polygon, TShape };

        public const ShapeType Shape2Use = ShapeType.Polygon;

        private bool isTracking;

        public enum Transformations { Scale, Yaw, Roll, Pitch, X, Y, Z };
        public enum Axis { X, Y };
        public HeadTracker(int cameraIndex = 0)
        {
            // Initialize the camera
            capture = new VideoCapture(cameraIndex);
            capture.Set(VideoCaptureProperties.Fps, CaptureFps); // Adjust as needed
            capture.FrameHeight = FRAMEHEIGHT;
            capture.FrameWidth = FRAMEWIDTH;
            Console.WriteLine("Size: " + capture.FrameHeight + " - " + capture.FrameWidth);
        }

        // Detect contours or blobs (these would correspond to your LEDs/markers)
        public Triangle curTriangle { get; private set; }
        public Polygon curPolygon { get; private set; }
        public TShape curTShape { get; private set; }

        public Triangle centerTriangle { get; private set; }
        public Polygon centerPolygon { get; private set; }
        public TShape centerTShape { get; private set; }
        public static Point2f[] RP(Point2f[] points, float rollAngle)
        {
            // Calculate center point
            float centerX = (points[0].X + points[1].X + points[2].X) / 3;
            float centerY = (points[0].Y + points[1].Y + points[2].Y) / 3;

            // Convert angle to radians
            double angleRad = rollAngle * Math.PI / 180;

            // Rotate points around center
            Point2f[] rotatedPoints = new Point2f[3];
            for (int i = 0; i < 3; i++)
            {
                float dx = points[i].X - centerX;
                float dy = points[i].Y - centerY;

                rotatedPoints[i] = new Point2f(
                    centerX + (float)(dx * Math.Cos(angleRad) - dy * Math.Sin(angleRad)),
                    centerY + (float)(dx * Math.Sin(angleRad) + dy * Math.Cos(angleRad))
                );
            }

            return rotatedPoints;
        }

        public static Point2f[] PP(Point2f[] points, float angleDegrees)
        {
            // Use bottom center as pivot point
            float baseX = (points[1].X + points[2].X) / 2;  // bottom center X
            float baseY = points[1].Y;  // bottom Y (since both bottom points have same Y)

            double angleRad = angleDegrees * Math.PI / 180;
            float scale = (float)Math.Cos(angleRad);

            Point2f[] pitchedPoints = new Point2f[3];

            // Keep bottom points fixed
            pitchedPoints[1] = points[1];  // right
            pitchedPoints[2] = points[2];  // left

            // Calculate top point position
            float height = baseY - points[0].Y;  // original height
            float newHeight = height * scale;    // compressed height

            // Move top point
            pitchedPoints[0] = new Point2f(
                points[0].X,
                baseY - newHeight  // move up from base by new height
            );

            return pitchedPoints;
        }

        public static Point2f[] YP(Point2f[] points, float angleDegrees)
        {
            float centerX = (points[0].X + points[1].X + points[2].X) / 3;
            float centerY = (points[0].Y + points[1].Y + points[2].Y) / 3;

            double angleRad = -angleDegrees * Math.PI / 180;  // Negative to reverse direction
            float scale = (float)Math.Cos(angleRad);  // Foreshortening effect

            Point2f[] yawedPoints = new Point2f[3];
            for (int i = 0; i < 3; i++)
            {
                float dx = points[i].X - centerX;
                // X coordinates compress based on angle, direction reversed
                yawedPoints[i] = new Point2f(
                    centerX + (dx * scale),
                    points[i].Y
                );
            }
            return yawedPoints;
        }

        public bool HasCenter { get; set; } = false;

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
                    else if (k.Key == ConsoleKey.C)
                    {
                        Console.WriteLine("Prediction Cleared");
                        PoseTransformation.ClearOffsets();
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

                //if (frame.Empty() || frame.IsThesame(prevFrame))
                //{
                //    continue;
                //}

                var ledPoints = ExtractLedPoints(frame);

                displayFrame = frame.EmptyClone();
                CloneFrame = frame.EmptyClone();

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

                        curTShape.ShowCurrentShape(displayFrame);

                        break;
                    default:
                        break;
                }

                if (HasCenter)
                {
                    ShowCenterTriangle(displayFrame);
                }

                ShowHeadPose(displayFrame);

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
                Cv2.GaussianBlur(grayFrame, grayFrame, new Size(5, 5), 0);

                Cv2.Threshold(grayFrame, grayFrame, 10, 255, ThresholdTypes.Binary);

                //Cv2.ImShow("Gray Frame", grayFrame);

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

                    Point2f center = new Point2f((int)(moments.M10 /    moments.M00), (int)(moments.M01 / moments.M00));

                    pulses.Add((center, area));
                }
            }

            // if 2 pulses are too close together remove the one with the smaller area

            float minDistance = 10;

            if(pulses.Count > 4)
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
                        Point2f pos = new Point2f(10, 20);

                        centerTShape.PrintData(displayFrame, pos);


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
                // check gpt, but if we computer pnp on both the center and the current points, we can get the difference between the two and use that to determine the angle of rotation
                //PoseTransformation.EstimateTransformation(points, out Point3d r2, out Point3d t2);

                PoseTransformation.EstimateTransformation2(points, out Point3d r2, out Point3d t2);

                //Cv2.PutText(displayFrame, "Rotation: " + r.R2P(), 
                //    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                //Cv2.PutText(displayFrame, "Translation: " + t.R2P(), 
                //    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                c += 2;

                Cv2.PutText(displayFrame, "Rotation2: " + r2.R2P(),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                Cv2.PutText(displayFrame, "Translation2: " + t2.R2P(),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                c++;

                Cv2.PutText(displayFrame, "Rotation2: " + r2.R2P(),
            start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                Cv2.PutText(displayFrame, "Translation2: " + t2.R2P(),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                SendData(r2);

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



        private static UdpClient udpClient;
        private static readonly string localhost = "127.0.0.1";
        private static readonly int port = 9876;
        public static void SendData(Point3d r)
        {
            string pitch = r.X.ToString("F2");
            string yaw = r.Y.ToString("F2");
            string roll = r.Z.ToString("F2");

            if (udpClient == null)
            {
                udpClient = new UdpClient();
            }

            try
            {
                // Format: "pitch,yaw,roll,tx,ty,tz"
                string data = $"{pitch},{yaw},{roll}";
                byte[] bytes = Encoding.UTF8.GetBytes(data);
                udpClient.Send(bytes, bytes.Length, localhost, port);
            }
            catch (Exception e)
            {
                Console.WriteLine($"UDP Send Error: {e.Message}");
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