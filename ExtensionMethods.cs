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

        public static Point R2P(this Point2f p)
        {
            return new Point(Math.Round(p.X), Math.Round(p.Y));
        }

        public static Point2f Abs(this Point2f p)
        {
            return new Point2f(Math.Abs(p.X), Math.Abs(p.Y));
        }

        public static T Clamp<T>(this T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0) return min;
            else if (val.CompareTo(max) > 0) return max;
            else return val;
        }
    }
}
