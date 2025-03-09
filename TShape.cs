using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using static Headtracker_Console.ExtensionMethods;
using static Headtracker_Console.HeadTracker;

namespace Headtracker_Console
{
    public class TShape
    {

        public Point2f[] Points;
        public Point2f[] PointsWithCenter
        {
            get
            {
                List<Point2f> points = new List<Point2f>();

                foreach (var point in Points)
                {
                    points.Add(point);
                }

                points.Add(Centroid);

                return points.ToArray();
            }
        }
        public Point2f Centroid
        {
            get
            {
                float x = 0;
                float y = 0;

                foreach (var point in Points)
                {
                    x += point.X;
                    y += point.Y;
                }

                return new Point2f(x / Points.Length, y / Points.Length);

            }
        }
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

                return true;
            }
        }
        public Mat PointsMatrix
        {
            get
            {
                int rows = Points.Length;

                Mat m = new Mat(rows, 2, MatType.CV_32FC1);

                for (int i = 0; i < rows; i++)
                {
                    m.Set(i, 0, Points[i].X);
                    m.Set(i, 1, Points[i].Y);
                }

                return m;
            }
        }

        public TShape(List<Point2f> leds)
        {
            if(leds.Count < 3)
            {
                Points = new Point2f[0];
                return;
            }

            List<Point2f> points = new List<Point2f>(leds);

            // sort points from most left to most right

            points.Sort((a, b) => a.X.CompareTo(b.X));

            Point2f left = points[0];
            Point2f top = points[1];

            if(top.Y > left.Y)
            {
                Point2f temp = top;
                top = left;
                left = temp;
            }

            Point2f right = points[2];

            Points = new Point2f[] { left, top, right };
        }
        public void Translate(Point2f p)
        {
            List<Point2f> newPoints = new List<Point2f>();
            foreach (var point in Points)
            {
                newPoints.Add(point - p);
            }

            Points = newPoints.ToArray();
        }
        public void TranslateTo(Point2f center)
        {
            // move current triangle to new center

            Point2f cDiff = center - Centroid;

            List<Point2f> newPoints = new List<Point2f>();
            foreach (var point in Points)
            {
                newPoints.Add(point - cDiff);
            }

            Points = newPoints.ToArray();
        }
        public void DrawShape(Mat frame, Scalar sl, bool showLengths = true)
        {
            Point2f px = new Point2f(20, 0);
            Point2f xp = new Point2f(70, 0);
            Point2f py = new Point2f(0, 30);

            int n = Points.Length;
            //Scalar sl = Scalar.White;

            foreach (var point in Points)
            {
                Cv2.Circle(frame, point.R2P(), 2, sl, 2);
            }

            // Draw lines in inverse T shape and shows their distances

            Point2f mid = MidPoint2f(Points[0], Points[2]);
            Point2f mid2 = new Point2f(Points[1].X, mid.Y);

            Point2f P0 = Points[0];    // Left base
            Point2f P2 = Points[2];    // Right base
            Point2f P1 = Points[1];    // Top point
            // Vector math with Point2f
            Point2f baseVector = P2 - P0;
            Point2f topVector = P1 - P0;

            // Projection formula
            float dot = (topVector.X * baseVector.X + topVector.Y * baseVector.Y);
            float baseLengthSq = (baseVector.X * baseVector.X + baseVector.Y * baseVector.Y);
            float t = dot / baseLengthSq;

            // Intersection point
            Point2f intersection = new Point2f(
                P0.X + baseVector.X * t,
                P0.Y + baseVector.Y * t
            );

            Cv2.Line(frame, Points[0].R2P(), Points[2].R2P(), sl, 1);
            Cv2.Line(frame, Points[1].R2P(), intersection.R2P(), sl, 1);

            string height = Point2f.Distance(Points[1], intersection).ToString("0.00");
            string width = Point2f.Distance(Points[0], Points[2]).ToString("0.00");   
            Cv2.PutText(frame, width, (mid + py).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, height, (Centroid + px).R2P(), HersheyFonts.HersheyPlain, 1, sl);
        }

        public void DrawCentriod(Mat frame)
        {
            Cv2.Circle(frame, Centroid.R2P(), 2, Scalar.White, 2);
        }
        public void PrintData(Mat frame, Point2f pos)
        {
            Point2f py = new Point2f(0, 20);
            Scalar sl = Scalar.White;

            Point2f baseVector, heightVector;
            Point2f mid = MidPoint2f(Points[0], Points[2]);

            baseVector = Points[2] - Points[0];
            heightVector = Points[1] - mid;

            Cv2.PutText(frame, "Base Vector: " + baseVector.ToString2(), pos.R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Height Vector: " + heightVector.ToString2(), (pos + py).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            float ratio = heightVector.Magnitude() / baseVector.Magnitude();

            Cv2.PutText(frame, "Base Height Ratio: " + ratio.ToString(), (pos + py.Multiply(5)).R2P(), HersheyFonts.HersheyPlain, 1, sl);
        }

        public void ShowCurrentShape(Mat displayFrame)
        {
            try
            {
                DrawShape(displayFrame, Scalar.Red);
                DrawCentriod(displayFrame);

                Point2f pos = new Point2f(10, 80);
                PrintData(displayFrame, pos);

            }
            catch (Exception)
            {

            }
        }

    }
}
