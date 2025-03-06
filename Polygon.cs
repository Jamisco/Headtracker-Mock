using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using static Headtracker_Console.HeadTracker;
using static Headtracker_Console.ExtensionMethods;

namespace Headtracker_Console
{
    public class Polygon
    {
        public Point2f[] Points;
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

        public Polygon(List<Point2f> leds)
        {
            List<Point2f> points = new List<Point2f>(leds);

            // sort points from most left to most right

            points.Sort((a, b) => a.X.CompareTo(b.X));

            points = points.Take(4).ToList();

            this.Points = points.ToArray();
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

            // Draw lines and shows their distances

            for (int i = 0; i < n; i++)
            {

                Point2f a = Points[i];
                Point2f b = Points[(i + 1) % n];

                Cv2.Line(frame, a.R2P(), b.R2P(), sl, 2);

                int c = lineVector(a, b);
                string d = Point2f.Distance(a, b).ToString("F2");
                Point2f mid = MidPoint2f(a, b);

                if (c == 0)
                {
                    if (Points[i].Y > Centroid.Y)
                    {
                        Cv2.PutText(frame, d, (mid + py).R2P(), HersheyFonts.HersheyPlain, 1, sl);
                    }
                    else
                    {
                        Cv2.PutText(frame, d, (mid - py).R2P(), HersheyFonts.HersheyPlain, 1, sl);
                    }
                }
                else
                {
                    if (Points[i].X > Centroid.X)
                    {
                        Cv2.PutText(frame, d, (mid + px).R2P(), HersheyFonts.HersheyPlain, 1, sl);
                    }
                    else
                    {
                        Cv2.PutText(frame, d, (mid - xp).R2P(), HersheyFonts.HersheyPlain, 1, sl);
                    }
                }
            }

            //if (showLengths)
            //{
            //    for (int i = 0; i < n; i++)
            //    {
            //        int j = (i + 1) % n;
            //        Cv2.PutText(frame, Point2f.Distance(Points[i], Points[j]).ToString("F2"), (MidPoint2f(Points[i], Points[j]) - xp).R2P(), HersheyFonts.HersheyPlain, 1, sl);
            //    }
            //}

            int lineVector(Point2f i, Point2f j)
            {
                Point2f diff = i - j;

                diff = diff.Abs();

                if (diff.X > diff.Y)
                {
                    return 0;
                }
                else
                {
                    return 1;
                }
            }
        }
        public void DrawCentriod(Mat frame)
        {
            Cv2.Circle(frame, Centroid.R2P(), 2, Scalar.White, 2);
        }
        public void PrintData(Mat frame)
        {
            // dont need
        }

        public void ShowCurrentPolygon(Mat displayFrame)
        {
            try
            {
                DrawShape(displayFrame, Scalar.Red);
                DrawCentriod(displayFrame);
            }
            catch (Exception)
            {

            }
        }

    }
}
