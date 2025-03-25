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
    public class TShape
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

        public Point2f TopBaseIntercept
        {
            get
            {
                return GetPerpendicularPoint(Points[1], Points[0], Points[2]);
            }
        }
        public Point2f HeightCenterIntercept
        {
            get
            {
                return GetPerpendicularPoint(Centroid, Points[1], TopBaseIntercept);
            }
        }

        public Point2f BaseCentroid
        {
            get
            {
                return MidPoint2f(Points[0], Points[2]);
            }
        }
        public Point2f TopCentroid
        {
            get
            {
                return MidPoint2f(Points[1], TopBaseIntercept);
            }
        }
        public Point2f HeightVector
        {
            get
            {
                return TopBaseIntercept - Points[1];
            }
        }
        public Point2f BaseVector
        {
            get
            {
                return Points[2] - Points[0];
            }
        }
        public Point2f CenterVector
        {
            get
            {
                return Centroid - TopCentroid;
            }
        }


        public float Height
        {
            get
            {
                return (float)Point2f.Distance(Points[1], TopBaseIntercept);
            }
        }

        public float Width
        {
            get
            {
                return (float)Point2f.Distance(Points[0], Points[2]);
            }
        }

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

        public TShape(List<Point2f> leds)
        {
            if (leds == null ||  leds.Count < 3)
            {
                Points = new Point2f[0];
                return;
            }

            List<Point2f> points = new List<Point2f>(leds);

            // sort points from most left to most right

            points.Sort((a, b) => a.Y.CompareTo(b.Y));

            Point2f top = points[0];

            Point2f left, right;

            if (points[1].X > points[2].X)
            {
                right = points[1];
                left = points[2];
            }
            else
            {
                right = points[2];
                left = points[1];
            }

            Points = new Point2f[] { left, top, right };
        }

        [JsonConstructor]
        public TShape(Point2f[] points)
        {
            Points = points;
        }


        //public TShape UndoVector(TShape cur, TShape center)
        //{
        //    Point2f h
        //}


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
                newPoints.Add(point + cDiff);
            }

            Points = newPoints.ToArray();
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

            Point2f mid = MidPoint2f(Points[0], Points[2]);
            Point2f mid2 = new Point2f(Points[1].X, mid.Y);

            int ls = 2;

            Cv2.Line(frame, Points[0].R2P(), Points[2].R2P(), sl, ls);
            Cv2.Line(frame, Points[1].R2P(), TopBaseIntercept.R2P(), sl, ls);
            Cv2.Line(frame, Centroid.R2P(), HeightCenterIntercept.R2P(), sl, ls);

            Cv2.Line(frame, Points[0].R2P(), Points[1].R2P(), sl, ls);
            Cv2.Line(frame, Points[1].R2P(), Points[2].R2P(), sl, ls);


            string height = Point2f.Distance(Points[1], TopBaseIntercept).ToString("0.00");
            string width = Point2f.Distance(Points[0], Points[2]).ToString("0.00");

            string cWdith = Point2f.Distance(Centroid, HeightCenterIntercept).ToString("0.00");

            string ltDistance = Point2f.Distance(Points[0], Points[1]).ToString("0.00");
            string rtDistance = Point2f.Distance(Points[1], Points[2]).ToString("0.00");

            string cHeight = (Centroid.Y - TopCentroid.Y).ToString("0.00");

            float ts = 1.3f;

            if(showLengths == false)
            {
                return;
            }

            Cv2.PutText(frame, width, (mid + py).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Cv2.PutText(frame, height, (Points[1] + px).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Cv2.PutText(frame, cWdith, (Centroid + px).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Cv2.PutText(frame, cHeight, (Centroid - xp).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Point2f midp = MidPoint2f(Points[0], Points[1]);

            Cv2.PutText(frame, ltDistance, (midp - xp2 * 4).R2P(), HersheyFonts.HersheyPlain, ts, sl);

            Cv2.PutText(frame, rtDistance, (MidPoint2f(Points[1], Points[2]) + xp2).R2P(), HersheyFonts.HersheyPlain, ts, sl);
        }

        public void AdjustBaseVector(Point2f baseDiff, Point2f heightDiff)
        {
           

        }
        public void AdjustHeightVector(Point2f vectorDiff)
        {
            Point2f hv = Points[1] - TopBaseIntercept;

            vectorDiff = vectorDiff.Multiply(-1);

            Point2f bo = hv + vectorDiff;

            Point2f newTop = TopBaseIntercept + bo;

            Points[1] = newTop;
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
            Cv2.PutText(frame, "Base Vector: " + BaseVector.R2P().ToString2(), pos.R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Height Vector: " + HeightVector.R2P().ToString2(), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Center Vector: " + CenterVector.R2P().ToString2(), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            c += 2;

            float nx = CameraProperties.objectSize.X;
            float ny = CameraProperties.objectSize.Y;
            float nz = CameraProperties.objectSize.Z;

            float sx = CameraProperties.objScreenSize.X;
            float sy = CameraProperties.objScreenSize.Y;

            float total = Height + Width;
            float hr = Height / total;
            float wr = Width / total;
            float actualRatio = ny / (nx + ny);




            float pti = CameraProperties.pixelToInch;

            // dfc = distance from cam
            float curBaseDFC = Width / pti;

            float t2l = (float)Point2f.Distance(TopBaseIntercept, Points[0]) / Width;
            float t2r = (float)Point2f.Distance(TopBaseIntercept, Points[2]) / Width;

            float choosen = 1 - t2l;

            // the depth the forward points are at
            float baseDepth = 20;

            float depthInc = (float)Math.Cos(Math.PI * (choosen));

            // the depth from forward point to back point
            float defaultDFC = (sx * (nz / nx)) / pti;
            float topDFC = baseDepth + defaultDFC - (defaultDFC * depthInc);


            Cv2.PutText(frame, "Height Ratio: " + hr.ToString("0.00"), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Width Ratio: " + wr.ToString("0.00"), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Actual Height Ratio: " + (actualRatio).ToString("0.00"), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Top DFC: " + topDFC.ToString("0.00"), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

            Cv2.PutText(frame, "Def Top DFC: " + defaultDFC.ToString("0.00"), (pos + py.Multiply(c++)).R2P(), HersheyFonts.HersheyPlain, 1, sl);

        }

        public List<string> GetPrintData()
        {
            List<string> data = new List<string>();

            data.Add("Base Vector: " + BaseVector.ToString2());
            data.Add("Height Vector: " + HeightVector.ToString2());
            data.Add("Center Vector: " + CenterVector.ToString2());

            return data;
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

        public void RotateShaoe(float angle)
        {
            Point2f center = Centroid;

            List<Point2f> np = new List<Point2f>();

            foreach (Point2f p in Points)
            {
              Point2f pp = PoseTransformation.Rotate(p, center, angle);

                np.Add(pp);
            }

            Points = np.ToArray();
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
