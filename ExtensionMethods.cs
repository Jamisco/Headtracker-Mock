using OpenCvSharp;
using System;

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

        public static string R2P(this Point3d p)
        {
            Point3d b = new Point3d(Math.Round(p.X), Math.Round(p.Y), Math.Round(p.Z));

            return "" + b.X + ", " + b.Y + ", " + b.Z;
        }

        public static double Clamp(this double value, double min, double max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
        }

        // Optional: Add float version if needed
        public static float Clamp(this float value, float min, float max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
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
    }
}
