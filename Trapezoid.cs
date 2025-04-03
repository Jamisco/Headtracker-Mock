using MathNet.Numerics;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json.Serialization;
using static Headtracker_Console.ExtensionMethods;
using static Headtracker_Console.HeadTracker;

namespace Headtracker_Console
{
    [Serializable]
    public class Trapezoid
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

        public Point2f BaseCentroid
        {
            get
            {
                return MidPoint2f(Points[0], Points[3]);
            }
        }
        public Point2f TopCentroid
        {
            get
            {
                return MidPoint2f(Points[1], Points[2]);
            }
        }


        public float Height
        {
            get
            {
                return (float)Point2f.Distance(TopCentroid, BaseCentroid);
            }
        }

        public float Width
        {
            get
            {
                return (float)Point2f.Distance(Points[0], Points[3]);
            }
        }

        public bool IsValid
        {
            get
            {
                return IsValidPoints(Points);
            }
        }
        //public readonly Mat PointsMatrix
        //{
        //    get
        //    {
        //        int rows = Points.Length;

        //        Mat m = new Mat(rows, 2, MatType.CV_32FC1);

        //        for (int i = 0; i < rows; i++)
        //        {
        //            m.Set(i, 0, Points[i].X);
        //            m.Set(i, 1, Points[i].Y);
        //        }

        //        return m;
        //    }
        //}

        public static bool IsValidPoints(Point2f[] points)
        {
            if (points.Length != 4)
            {
                return false;
            }

            foreach (var point in points)
            {
                if (point.X < 0 || point.X >= FRAMEWIDTH || point.Y < 0 || point.Y >= FRAMEHEIGHT)
                {
                    return false;
                }
            }

            return true;
        }

        public Trapezoid(List<Point2f> leds)
        {
            if (leds == null || leds.Count < 4)
            {
                Points = new Point2f[0];
                return;
            }

            // Sort points from left to right
            leds.Sort((a, b) => a.X.CompareTo(b.X));

            // Identify the top and bottom points for the left and right sides
            Point2f[] leftPoints = leds.Take(2).OrderBy(p => p.Y).ToArray();
            Point2f[] rightPoints = leds.Skip(2).Take(2).OrderBy(p => p.Y).ToArray();

            // Assign points in the correct order: bottom-left, top-left, top-right, bottom-right
            Points = new Point2f[]
            {
                leftPoints[0], // bottom-left
                leftPoints[1], // top-left
                rightPoints[1], // top-right
                rightPoints[0]  // bottom-right
            };
        }

        [JsonConstructor]
        public Trapezoid(Point2f[] points)
        {
            Points = points;
        }
        public void DrawShape(Mat frame, Scalar sl, bool showLengths = true)
        {
            Point2f px = new Point2f(20, 0);
            Point2f xp = new Point2f(70, 0);
            Point2f py = new Point2f(0, 30);
            Point2f xp2 = new Point2f(20, 0);

            int n = Points.Length;
            //Scalar sl = Scalar.White;

            foreach (var point in Points)
            {
                Cv2.Circle(frame, point.R2P(), 2, sl, 2);
            }

            // Draw lines in inverse T shape and shows their distances

            int ls = 2;

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;

                Cv2.Line(frame, Points[i].R2P(), Points[j].R2P(), sl, ls);
            }

            Point2f mid = MidPoint2f(Points[0], Points[2]);
            Point2f mid2 = new Point2f(Points[1].X, mid.Y);

            string height = Height.ToString("0.00");
            string width = Width.ToString("0.00");

            float ts = 1.3f;

            if (showLengths == false)
            {
                return;
            }

            Cv2.PutText(frame, width, (mid + py).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Cv2.PutText(frame, height, (Points[1] + px).R2P(), HersheyFonts.HersheyPlain, ts, sl);
        }
        public void DrawCentriod(Mat frame)
        {
            Cv2.Circle(frame, Centroid.R2P(), 2, Scalar.White, 2);

            Cv2.Circle(frame, TopCentroid.R2P(), 2, Scalar.Yellow, 2);


        }
        public void PrintData(Mat frame, Point2f pos)
        {
            Point2f py = new Point2f(0, 20);
            int c = 1;
            Scalar sl = Scalar.White;

            Cv2.PutText(frame, "Centroid: " + Centroid.R2P().ToString2(), pos.R2P(), HersheyFonts.HersheyPlain, 1, sl);
        }
        public void ShowCurrentShape(Mat displayFrame, Point2f printPos)
        {
            try
            {
                DrawShape(displayFrame, Scalar.Red);
                DrawCentriod(displayFrame);

                PrintData(displayFrame, printPos);
            }
            catch (Exception)
            {

            }
        }
        /// <summary>
        /// Gets the point that is perpendicular to the line from p1 to p2.
        /// The returned point when combined with line p1 to p2 will form a right angle.
        /// </summary>
        /// <param name="p0"></param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public Point2f GetPerpendicularPoint(Point2f p0, Point2f p1, Point2f p2)
        {
            Point2f baseVector = p2 - p1;
            Point2f topVector = p0 - p1;

            // Projection formula
            float dot = (topVector.X * baseVector.X + topVector.Y * baseVector.Y);
            float baseLengthSq = (baseVector.X * baseVector.X + baseVector.Y * baseVector.Y);
            float t = dot / baseLengthSq;

            // Intersection point
            Point2f intersection = new Point2f(
                p1.X + baseVector.X * t,
                p1.Y + baseVector.Y * t
            );

            return intersection;
        }

    }
}
