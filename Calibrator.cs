using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static Headtracker_Console.HeadTracker;

namespace Headtracker_Console
{
    public class Calibrator
    {
        public void DrawGraph(Mat frame, Point2f centriod)
        {
            int circleCount = 15;
            int ra = 10;

            int height = frame.Height;
            int width = frame.Width;

            int hs = height / circleCount;
            int ws = width / circleCount;

            int x = (int)centriod.X;
            int y = (int)centriod.Y;

            Point x1 = new Point(0, y);
            Point x2 = new Point(width, y);

            Point y1 = new Point(x, 0);
            Point y2 = new Point(x, height);

            Cv2.Line(frame, x1, x2, Scalar.White, 1);
            Cv2.Line(frame, y1, y2, Scalar.White, 1);

            Cv2.Circle(frame, centriod.R2P(), ra, Scalar.White, 2);

            for (int i = 0; i < circleCount; i++)
            {
                Point xp = new Point(x + (i * ws), y);
                Point nxp = new Point(x - (i * ws), y);

                Point yp = new Point(x, y + (i * hs));
                Point nyp = new Point(x, y - (i * hs));

                Cv2.Circle(frame, xp, ra, Scalar.Red);
                Cv2.Circle(frame, nxp, ra, Scalar.Red);

                Cv2.Circle(frame, yp, ra, Scalar.Red);
                Cv2.Circle(frame, nyp, ra, Scalar.Red);
            }

        }

        public enum CalibrateType { Pitch, Yaw, Roll };

        public bool HasCalibration
        {
            get
            {
                return RollHolder.Negative.Last() != null;
            }
        }

        public bool CalibratingPitch { get; set; } = false;
        public bool CalibratingYaw { get; set; } = false;
        public bool CalibratingRoll { get; set; } = false;
        public void CalibrateKeyPress(CalibrateType t)
        {
            switch (t)
            {
                case CalibrateType.Pitch:
                    CalibratingPitch = !CalibratingPitch;
                    break;
                case CalibrateType.Yaw:
                    CalibratingYaw = !CalibratingYaw;
                    break;
                case CalibrateType.Roll:
                    CalibratingRoll = !CalibratingRoll;
                    break;
                default:
                    break;
            }
        }

        bool hasCenter = false;
        private TShape Center;
        public void UpdateCenter(TShape center)
        {
            Center = center;

            if(!hasCenter)
            {
                PitchHolder = new ShapeHolder(center, Axis.Y);
                YawHolder = new ShapeHolder(center, Axis.X);
                RollHolder = new ShapeHolder(center, Axis.X);
            }
        }

        public void GetUnitVector(out Point3f r, out Point3f t)
        {
            if (CalibratingPitch)
            {
                PitchHolder.GetUnitVector(out r, out t);
            }
            else if (CalibratingYaw)
            {
                YawHolder.GetUnitVector(out r, out t);
            }
            else if (CalibratingRoll)
            {
                RollHolder.GetUnitVector(out r, out t);
            }
            else
            {
                RollHolder.GetUnitVector(out r, out t);
            }
        }

        private ShapeHolder PitchHolder;
        private ShapeHolder YawHolder;
        private ShapeHolder RollHolder;

        public void AddShapeState(TShape shape)
        {
            if (shape.Points.Length < 3)
            {
                return;
            }

            if (CalibratingPitch)
            {
                PitchHolder.AddShape(shape);
            }
            else if (CalibratingYaw)
            {
                YawHolder.AddShape(shape);
            }
            else if (CalibratingRoll)
            {
                RollHolder.AddShape(shape);
            }
            else
            {
               // Console.WriteLine("No Calibration Type Selected!");
            }
        }

        public struct ShapeHolder
        {
            public List<TShape> Positive;
            public List<TShape> Negative;
            public TShape CenterShape;
            public Axis shapeAxis;
            public ShapeHolder(TShape centerShape, Axis axis)
            {
                CenterShape = centerShape;
                shapeAxis = axis;

                Positive = new List<TShape>();
                Negative = new List<TShape>();
            }

            public void AddShape(TShape shape)
            {
                float num = 0;

                num = (shapeAxis == Axis.X) ? shape.Centroid.X : shape.Centroid.Y;

                if (shapeAxis == Axis.X)
                {
                    if (num > CenterShape.Centroid.X)
                    {
                        AddPos(shape, num);
                    }
                    else
                    {
                        AddNeg(shape, num);
                    }
                }
                else
                {
                    if (num > CenterShape.Centroid.Y)
                    {
                        AddPos(shape, num);
                    }
                    else
                    {
                        AddNeg(shape, num);
                    }
                }
            }
            private void AddPos(TShape shape, float num)
            {
                if(Positive.Count == 0)
                {
                    Positive.Add(shape);
                    return;
                }

                TShape last = Positive.Last();

                float num2 = (shapeAxis == Axis.X) ? last.Centroid.X : last.Centroid.Y;

                if(num > num2)
                {
                    Positive.Add(shape);
                }
            }
            private void AddNeg(TShape shape, float num)
            {
                if(Negative.Count == 0)
                {
                    Negative.Add(shape);
                    return;
                }

                TShape last = Negative.Last();

                float num2 = (shapeAxis == Axis.X) ? last.Centroid.X : last.Centroid.Y;

                if (num < num2)
                {
                    Negative.Add(shape);
                }
            }

            public void GetUnitVector(out Point3f r, out Point3f t)
            {
                Point2f centerT = CenterShape.TopCentroid;
                TShape last = Negative.Last();

                PoseTransformation.GetPureRotation(CenterShape, last,
                    out Point3f r2);

                Point2f tDiff = last.TopCentroid - centerT;

                float biggestT = Math.Max(Math.Abs(tDiff.X), Math.Abs(tDiff.Y));

                float roll = (float)r2.Z;

                r = new Point3f(r2.X / roll, r2.Y / roll, r2.Z / roll);
                t = new Point3f(tDiff.X / roll, tDiff.Y / roll, 0);
            }
        }
    }
}
