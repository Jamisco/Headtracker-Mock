using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Headtracker_Console
{
    public static class PosePredictor
    {
        public static void PredictPose(Point2f[] center, Point2f[] cur, 
            out Mat rvec, Mat tvec)
        {
            // get the centroid of the points
            Point2f centerCentriod = GetCentriod(center);
            Point2f curCentriod = GetCentriod(center);

            float centerHeight = (float)Point2f.Distance(center[1], center[0]);
            float centerWidth = (float)Point2f.Distance(center.Last(), center[0]);

            float curHeight = (float)Point2f.Distance(cur[1], cur[0]);
            float curWidth = (float)Point2f.Distance(cur.Last(), cur[0]);

            float maxAngle = 90;
            float maxXDistance = 100;
            float maxYDistance = 100;

            var hDiff = Math.Abs(curHeight - centerHeight);
            var wDiff = Math.Abs(curWidth - centerWidth);

            float predictedPitch = (hDiff / centerHeight) * maxAngle;
            float predictedYaw = (wDiff / centerWidth) * maxAngle;
            float predictedRoll = (float)Math.Atan2(curCentriod.Y - centerCentriod.Y,
                                                  curCentriod.X - centerCentriod.X);

            // add the predicted values to the rvec
            rvec = new Mat(3, 1, MatType.CV_64FC1);

            rvec.Set(0, 0, predictedPitch);
            rvec.Set(1, 0, predictedYaw);
            rvec.Set(2, 0, predictedRoll);

            tvec = new Mat(3, 1, MatType.CV_64FC1);
            // set to zeroes

            tvec.Set(0, 0, 0);
            tvec.Set(1, 0, 0);
            tvec.Set(2, 0, 0);
        }

        private static Point2f GetCentriod(Point2f[] points)
        {
            Point2f centroid = new Point2f(0, 0);

            foreach (var point in points)
            {
                centroid.X += point.X;
                centroid.Y += point.Y;
            }

            centroid.X /= points.Length;
            centroid.Y /= points.Length;

            return centroid;
        }
    }
}
