using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using static Headtracker_Console.HeadTracker;

namespace Headtracker_Console
{
    public class Calibrator
    {
        private ShapeHolder PitchHolder;
        private ShapeHolder YawHolder;
        private ShapeHolder RollHolder;

        public Calibrator()
        {
            RetrieveSaveData();
        }

        // its in bin/debug folder
        public static string savePath = "CalibrationData.txt";
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
        public enum RotationType { Pitch, Yaw, Roll };
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
        public void CalibrateKeyPress(RotationType t)
        {
            switch (t)
            {
                case RotationType.Pitch:
                    CalibratingPitch = !CalibratingPitch;
                    break;
                case RotationType.Yaw:
                    CalibratingYaw = !CalibratingYaw;
                    break;
                case RotationType.Roll:
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

            if (!hasCenter)
            {
                PitchHolder = new ShapeHolder(center, Axis.Y);
                YawHolder = new ShapeHolder(center, Axis.X);
                RollHolder = new ShapeHolder(center, Axis.X);
            }
        }
        public void GetUnitVector(RotationType calType, out UnitVector u)
        {
            switch (calType)
            {
                case RotationType.Pitch:
                    u = PitchHolder.GetUnitVector();

                    break;
                case RotationType.Yaw:
                    u = YawHolder.GetUnitVector();

                    break;
                case RotationType.Roll:
                    u = RollHolder.GetUnitVector();

                    break;
                default:
                    u = RollHolder.GetUnitVector();
                    break;
            }
        }
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

        static JsonSerializerOptions options = new JsonSerializerOptions
        {
            IncludeFields = true,
            IgnoreReadOnlyProperties = true,
            WriteIndented = true
        };

        private static string splitter = "\n\n-----\n\n";
        public void SaveCurrentData()
        {
            var pitchData = JsonSerializer.Serialize(PitchHolder, options);
            var yawData = JsonSerializer.Serialize(YawHolder, options);
            var rollData = JsonSerializer.Serialize(RollHolder, options);

            var data = $"{pitchData}{splitter}{yawData}{splitter}{rollData}{splitter}";

            System.IO.File.WriteAllText(savePath, data);

            Console.WriteLine("Calibration Data Saved");
        }
        public void ClearCurrentCalibration()
        {
            PitchHolder.Clear();
            YawHolder.Clear();
            RollHolder.Clear();
        }

        public void DrawCenterData(Mat frame)
        {
            Point2f curCenter = Center.TopCentroid;

            if(CalibratingPitch)
            {
                PitchHolder.DrawShapes(frame, curCenter);
            }
            else if(CalibratingYaw)
            {
                YawHolder.DrawShapes(frame, curCenter);
            }
            else if(CalibratingRoll)
            {
                RollHolder.DrawShapes(frame, curCenter);
            }
        }

        public void RetrieveSaveData()
        {
            if (!System.IO.File.Exists(savePath))
            {
                Console.WriteLine("No saved calibration data found.");
                return;
            }

            var data = System.IO.File.ReadAllText(savePath);
            var sections = data.Split(new[] { splitter }, StringSplitOptions.None);

            if (sections.Length < 3)
            {
                Console.WriteLine("Invalid calibration data format.");
                return;
            }

            try
            {
                PitchHolder = JsonSerializer.Deserialize<ShapeHolder>(sections[0], options);
                YawHolder = JsonSerializer.Deserialize<ShapeHolder>(sections[1], options);
                RollHolder = JsonSerializer.Deserialize<ShapeHolder>(sections[2], options);

                Console.WriteLine("Calibration Data Retrieved");

            }
            catch (Exception ex)
            {
                Console.WriteLine("Unable to Retrieve Calibration Data");

                if (PitchHolder.CenterShape == null)
                {
                    PitchHolder = new ShapeHolder();
                }

                if (YawHolder.CenterShape == null)
                {
                    YawHolder = new ShapeHolder();
                }

                if (RollHolder.CenterShape == null)
                {
                    RollHolder = new ShapeHolder();
                }
            }
        }

        /// <summary>
        /// Unit Vector denotes that for each angle of rotation for a given rotation type, the movement of other rotation is given.
        /// For example, if the pitch is rotated by 1 degree, the yaw and roll will move by the given translation and the point will translate by the given translation.
        /// </summary>
        public struct UnitVector
        {
            public Point3f Rotation;
            public Point3f Translation;

            public UnitVector(Point3f r, Point3f t)
            {
                Rotation = r;
                Translation = t;
            }
        }



        [Serializable]
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
                if (Positive.Count == 0)
                {
                    Positive.Add(shape);
                    return;
                }

                TShape last = Positive.Last();

                float num2 = (shapeAxis == Axis.X) ? last.Centroid.X : last.Centroid.Y;

                if (num > num2)
                {
                    Positive.Add(shape);
                }
            }
            private void AddNeg(TShape shape, float num)
            {
                if (Negative.Count == 0)
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
            public UnitVector GetUnitVector()
            {
                Point2f centerT = CenterShape.TopCentroid;
                TShape last = Negative.Last();

                PoseTransformation.GetPureRotation(CenterShape, last,
                    out Point3f r2);

                Point2f tDiff = last.TopCentroid - centerT;

                float biggestT = Math.Max(Math.Abs(tDiff.X), Math.Abs(tDiff.Y));

                float roll = (float)r2.Z;


                Point3f r = new Point3f(r2.X / roll, r2.Y / roll, r2.Z / roll);
                Point3f t = new Point3f(tDiff.X / roll, tDiff.Y / roll, 0);

                return new UnitVector(r, t);
            }


        public void DrawShapes(Mat frame, Point2f curCenter)
        {
            foreach (var shape in Positive)
            {
                Point2f shapeCenter = CenterShape.Centroid;

                Point2f diff = shape.Centroid - curCenter;

                Point2f adjPos = curCenter + diff;

                Cv2.Circle(frame, adjPos.R2P(), 2, Scalar.Red, 2);
            }

            foreach (var shape in Negative)
            {
                Point2f shapeCenter = CenterShape.Centroid;

                Point2f diff = shape.Centroid - curCenter;

                Point2f adjPos = curCenter + diff;

                Cv2.Circle(frame, adjPos.R2P(), 2, Scalar.Green, 2);
            }
        }
        public void Clear()
            {
                Positive?.Clear();
                Negative?.Clear();
            }
        }
    }
}
