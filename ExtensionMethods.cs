using OpenCvSharp;
using System;
using System.Linq;

namespace Headtracker_Console
{
    public static class ExtensionMethods
    {
        public static string ToString2(this Point p)
        {
            return "(" + p.X + ", " + p.Y + ")";
        }

        public static string ToString2(this Point2f p)
        {
            return "(" + p.X + ", " + p.Y + ")";
        }

        /// <summary>
        /// Rounds the point 
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public static string R2P(this Point3d p)
        {
            Point3d b = new Point3d(Math.Round(p.X), Math.Round(p.Y), Math.Round(p.Z));

            return "" + b.X + ", " + b.Y + ", " + b.Z;
        }

        /// <summary>
        /// Rounds the point 
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public static string R2P(this Point3f p)
        {
            Point3f b = new Point3f((float)Math.Round(p.X), (float)Math.Round(p.Y), (float)Math.Round(p.Z));

            return "" + b.X + ", " + b.Y + ", " + b.Z;
        }

        public static float RTD(float radians)
        {
            return (float)(radians * 180 / Math.PI);
        }

        public static double RTD(double radians)
        {
            return (double)(radians * 180 / Math.PI);
        }

        public static Point R2P(this Point2f p)
        {
            return new Point(Math.Round(p.X), Math.Round(p.Y));
        }

        public static Point2f Abs(this Point2f p)
        {
            return new Point2f(Math.Abs(p.X), Math.Abs(p.Y));
        }

        public static float Magnitude(this Point2f p)
        {
            return (float)Math.Sqrt(p.X * p.X + p.Y * p.Y);
        }

        public static T Clamp<T>(this T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0) return min;
            else if (val.CompareTo(max) > 0) return max;
            else return val;
        }

        public static bool IsThesame(this Mat cur, Mat other)
        {
            // subtracts the two matrices and checks if the sum of the differences is zero

            Mat diff = new Mat();

            if (cur.Size() == other.Size() && cur.Type() == other.Type())
            {
                Cv2.Absdiff(cur, other, diff);

                return Cv2.Sum(diff).Val0 == 0;

            }

            return false;
        }
        public static Point2f MidPoint2f(Point2f p1, Point2f p2)
        {
            return new Point2f((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
        }

        public static Point2f GetCentriod(Point2f[] points)
        {
            float x = 0;
            float y = 0;

            foreach (var point in points)
            {
                x += point.X;
                y += point.Y;
            }

            return new Point2f(x / points.Length, y / points.Length);
        }

        public static Point3f GetCentriod(Point3f[] points)
        {
            float x = 0;
            float y = 0;
            float z = 0;

            foreach (var point in points)
            {
                x += point.X;
                y += point.Y;
                z += point.Z;
            }

            return new Point3f(x / points.Length, y / points.Length, z / points.Length);
        }
    }
}
