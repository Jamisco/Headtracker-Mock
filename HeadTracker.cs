
using OpenCvSharp;
using System;
using System.Collections.Generic;
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

        private bool isTracking;

        public enum Transformations { Scale, Yaw, Roll, Pitch, X, Y, Z };
        public enum Axis { X, Y };

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

        public TShape prevTShape { get; private set; }
        public TShape curTShape { get; private set; }
        public TShape centerTShape { get; private set; }

        public Trapezoid prevTrap { get; private set; }
        public Trapezoid curTrap { get; private set; }
        public TShape centerTrap { get; private set; }


        public bool HasCenter { get; set; } = false;

        public void StartTracking()
        {
            Console.WriteLine("Press a key to Begin...");

            Console.ReadKey();

            isTracking = true;
            trackingThread = new Thread(TrackingLoop);
            keyPressThread = new Thread(ReadKey);

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
                        PoseTransformation.ClearOffsets();

                        Console.WriteLine("New Center Set");
                    }
                    else if (k.Key == ConsoleKey.X)
                    {
                        Console.WriteLine("Prediction Cleared");
                        PoseTransformation.ClearOffsets();
                    }
                    
                }
            }
        }

        Mat displayFrame;

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
                DrawCenterGraph();

                TShape t = new TShape(ledPoints);
                Trapezoid trap = new Trapezoid(ledPoints);

                if (trap.IsValid)
                {
                    if (curTrap == null)
                    {
                        prevTrap = curTrap;
                        curTrap = trap;
                    }
                    else if (curTrap != null && curTrap.IsValid)
                    {
                        float distance = (float)Point2f.Distance(curTrap.Centroid, trap.Centroid);

                        prevTrap = curTrap;
                        curTrap = trap;
                    }


                }

                //if (t.IsValid)
                //{
                //    if(curTShape == null)
                //    {
                //        prevTShape = curTShape;
                //        curTShape = t;

                //    }
                //    else if (curTShape != null && curTShape.IsValid)
                //    {
                //        float distance = (float)Point2f.Distance(curTShape.Centroid, t.Centroid);

                //        prevTShape = curTShape;

                //        curTShape = t;

                //        prevTrap = curTrap;
                //        curTrap = trap;
                //    }
                //}

                //curTShape.ShowCurrentShape(displayFrame, printStartPos);
                curTrap.ShowCurrentShape(displayFrame, printStartPos);
                printLine = 0;

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

                Cv2.Threshold(grayFrame, grayFrame, 50, 255, ThresholdTypes.Binary);


                // pass in the previous points and modify the method so that it favors points that are closer to the previous points.
                // also make it so that points that are within the blob of said closer points are thesame as the previous points.
                Cv2.FindContours(grayFrame, out curContours, out hierarchy, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

                var led = PointFromContours(grayFrame, curContours);

                if (SHOWGRAY)
                {
                    Cv2.ImShow("OG Image", frame);
                    Cv2.ImShow("Gray Frame", grayFrame);
                }

                return led;
            }
            catch (Exception ex)
            {
                return null;
            }
        }
        private List<Point2f> PointFromContours(Mat frame, Point[][] contours)
        {
            // Step 1: Calculate areas for all contours
            List<Tuple<Point[], double>> contourAreas = new List<Tuple<Point[], double>>();

            List<(Point2f, double)> pulses = new List<(Point2f, double)>();

            Cv2.PutText(frame, "Countours Found: " + contours.Length, new Point(0, 100), HersheyFonts.HersheyPlain, 1, Scalar.White);

            foreach (var contour in contours)
            {
                Moments moments = Cv2.Moments(contour);

                Point2f center = contour[0];

                Cv2.Circle(frame, center.R2P(), 10, Scalar.White, 2);
            }

            if(contours.Count() > 3)
            {
                int sds = 2; ;
            }

            if (contours.Length > 3 && prevTShape != null)
            {
                List<(float, Point[])> distance = new List<(float, Point[])>();

                foreach (var contour in contours)
                {
                    Moments moments = Cv2.Moments(contour);

                    Point2f con = new Point2f((int)(moments.M10 / moments.M00), (int)(moments.M01 / moments.M00));

                    float minD = float.MaxValue;

                    foreach (Point2f c in prevTShape.Points)
                    {
                        float d = (float)con.DistanceTo(c);

                        if (d < minD)
                        {
                            minD = d;
                        }
                    }

                    distance.Add((minD, contour));
                }

                distance.Sort((a, b) => a.Item1.CompareTo(b.Item1));

                distance = distance.Take(4).ToList();

                contours = distance.Select(d => d.Item2).ToArray();

            }


            //foreach (var contour in contours)
            //{
            //    Moments moments = Cv2.Moments(contour);

            //    Point2f center = contour[0];

            //    Cv2.Circle(frame, center.R2P(), 15, Scalar.White, 2);

            //}


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

            //if (HasCenter)
            //{
            //    centerTShape.DrawShape(displayFrame, Scalar.White, true);
            //    centerTShape.DrawCentriod(displayFrame);

            //    centerTShape.PrintData(displayFrame, printStartPos);
            //}

            if (HasCenter)
            {
                curTrap.DrawShape(displayFrame, Scalar.White, true);
                curTrap.DrawCentriod(displayFrame);

                curTrap.PrintData(displayFrame, printStartPos);
            }
        }
        private void ShowFrameCounter(Mat displayFrame)
        {
            Point bottom = new Point(0, 580);

            Cv2.PutText(displayFrame, "Frame Count: " + frameCount, bottom, HersheyFonts.HersheyPlain, 1, Scalar.White);
        }
        private void SetCenter()
        {
            centerTShape = curTShape;

            HasCenter = true;
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
        private void ShowHeadPose(Mat displayFrame)
        {
            Point3d r, t;

            Point start = new Point(0, 300);
            Point step = new Point(0, 20);
            int c = 0;

            Point2f[] points;

            //points = curTShape.Points;
            points = curTrap.Points;


            try
            {

                //PoseTransformation.EstimateTransformation5(displayFrame, curTShape,  out Point3f r2, out Point3f t2);

                PoseTransformation.EstimateTransformation6(displayFrame, curTrap, out Point3f r2, out Point3f t2);

                c += 2;

                Cv2.PutText(displayFrame, "Rotation2: " + r2.R2P(2),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

                Cv2.PutText(displayFrame, "Translation2: " + t2.R2P(),
                    start + (step * c++), HersheyFonts.HersheyPlain, 1, Scalar.White);

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