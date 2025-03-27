using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
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
                return PitchHolder.CenterShape != null && YawHolder.CenterShape != null && RollHolder.CenterShape != null;
            }
        }

        public bool CalibratingPitch { get; set; } = false;
        public bool CalibratingYaw { get; set; } = false;
        public bool CalibratingRoll { get; set; } = false;

        public bool IsCalibratingAny
        {
            get
            {
                return CalibratingPitch || CalibratingYaw || CalibratingRoll;
            }
        }
        public void CalibrateKeyPress(RotationType t)
        {
            switch (t)
            {
                case RotationType.Pitch:

                    if (IsCalibratingAny && CalibratingPitch == false)
                    {
                        return;
                    }

                    CalibratingPitch = !CalibratingPitch;
                    break;
                case RotationType.Yaw:

                    if (IsCalibratingAny && CalibratingYaw == false)
                    {
                        return;
                    }

                    CalibratingYaw = !CalibratingYaw;
                    break;
                case RotationType.Roll:

                    if (IsCalibratingAny && CalibratingRoll == false)
                    {
                        return;
                    }

                    CalibratingRoll = !CalibratingRoll;
                    break;
                default:
                    break;
            }
        }

        private TShape Center;
        public void UpdateCenter(TShape center, bool overwrite)
        {
            Center = center;

            if(overwrite)
            {
                InitHolders();
            }
        }

        private void InitHolders()
        {
            if (Center != null)
            {
                PitchHolder = new ShapeHolder(Center, RotationType.Pitch);
                YawHolder = new ShapeHolder(Center, RotationType.Yaw);
                RollHolder = new ShapeHolder(Center, RotationType.Roll);
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

        public void GetOrigin(RotationType calType, out Point2f o)
        {
            switch (calType)
            {
                case RotationType.Pitch:
                    PitchHolder.FindOrigin(out o);

                    break;
                case RotationType.Yaw:
                    YawHolder.FindOrigin(out o);


                    break;
                case RotationType.Roll:
                    RollHolder.FindOrigin(out o);
                    break;
                default:
                    RollHolder.FindOrigin(out o);

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
            WriteIndented = true,
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

            if (CalibratingPitch)
            {
                PitchHolder.DrawShapes(frame, curCenter);
            }
            else if (CalibratingYaw)
            {
                YawHolder.DrawShapes(frame, curCenter);
            }
            else if (CalibratingRoll)
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

                //if (PitchHolder.CenterShape == null)
                //{
                //    PitchHolder = new ShapeHolder();
                //}

                //if (YawHolder.CenterShape == null)
                //{
                //    YawHolder = new ShapeHolder();
                //}

                //if (RollHolder.CenterShape == null)
                //{
                //    RollHolder = new ShapeHolder();
                //}
            }
        }

        /// <summary>
        /// Unit Vector denotes that for each angle of rotation for a given rotation type, the movement of other rotation is given.
        /// For example, if the pitch is rotated by 1 degree, the yaw and roll will move by the given translation and the point will translate by the given translation.
        /// </summary>
        public struct UnitVector
        {
            public Point3f rotation;

            // polynomial regression model to fit translation values
            public (double[] xc, double[] yc) PtransCoeff;
            public (double[] xc, double[] yc) NtransCoeff;
            public UnitVector(Point3f pr, (double[], double[]) pt, (double[], double[]) nt)
            {
                rotation = pr;

                PtransCoeff = pt;
                NtransCoeff = nt;
            }

            public void OffsetRotation(Point3f pureRotation, out Point3f offset)
            {
                // truth be told a regresion model should also be used to offset rotations.
                // but for now, this will wait

                offset = new Point3f();
            }
            public Point2f ExpectedTranslation(float unit)
            {
                if(unit > 0)
                {
                    return PredictPoint(unit, PtransCoeff.xc, PtransCoeff.yc);

                }
                else
                {
                    return PredictPoint(unit, NtransCoeff.xc, NtransCoeff.yc);
                }
            }
            private Point2f PredictPoint(double angle, double[] coefficientsX, double[] coefficientsY)
            {
                double xNew = 0.0;
                double yNew = 0.0;

                for (int i = 0; i < coefficientsX.Length; i++)
                {
                    xNew += coefficientsX[i] * Math.Pow(angle, i);
                    yNew += coefficientsY[i] * Math.Pow(angle, i);
                }

                return new Point2f((float)xNew, (float)yNew);
            }
        }


        [Serializable]
        public struct ShapeHolder
        {
            public TShape CenterShape;
            public List<(float, TShape)> States;
            public RotationType RotationType;
            public ShapeHolder(TShape centerShape, RotationType rotType)
            {
                CenterShape = centerShape;
                RotationType = rotType;

                States = new List<(float, TShape)>();
            }

            public void AddShape(TShape shape)
            {
                float num = 0;

                PoseTransformation.GetPureRotation(CenterShape, shape, out Point3f r);

                num = (RotationType == RotationType.Pitch) ? r.X : (RotationType == RotationType.Yaw) ? r.Y : r.Z;

                bool sameNum = States.Any(x => x.Item1 == num);

                if (!sameNum)
                {
                    States.Add((num, shape));
                }
            }

            public void FindOrigin(out Point2f origin)
            {
                var shapes = States.ToList();

                if (shapes.Count < 2)
                    throw new ArgumentException("At least two shapes are required to find the origin.");

                // Variables to accumulate sums for solving the linear system
                float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0, sumYY = 0;
                int n = shapes.Count;

                // Iterate through the shapes and calculate the components
                for (int i = 0; i < n; i++)
                {
                    var (angle, shape) = shapes[i];

                    // Convert roll angle from degrees to radians
                    float theta = angle * (float)(Math.PI / 180.0);

                    float cosTheta = (float)Math.Cos(theta);
                    float sinTheta = (float)Math.Sin(theta);

                    // Get the centroid of the current shape
                    Point2f c = shape.TopCentroid;

                    // Accumulate the sums (circle fitting)
                    sumX += c.X;
                    sumY += c.Y;
                    sumXX += c.X * cosTheta + c.Y * sinTheta;
                    sumYY += c.Y * cosTheta - c.X * sinTheta;
                    sumXY += c.X * sinTheta - c.Y * cosTheta;
                }

                // Solve for the origin of rotation
                float originX = (sumX * sumYY - sumY * sumXY) / (sumXX - sumXY);
                float originY = (sumY * sumXX - sumX * sumXY) / (sumYY - sumXY);

                origin = new Point2f(originX / n, originY / n);

                Point2f cd = CenterShape.TopCentroid - CenterShape.TopBaseIntercept;


                origin = new Point2f(origin.X / cd.X, origin.Y / cd.Y);
            }
            public (double[], double[]) FitModel(List<(float, TShape)> shapes)
            {
                List<(double angle, Point2f point)> data = new List<(double angle, Point2f point)>();

                Point2f initDiff = States[0].Item2.Centroid - CenterShape.Centroid;
                // apply dead zone maybe?
                foreach (var state in shapes)
                {
                    data.Add((state.Item1, state.Item2.Centroid - CenterShape.Centroid - initDiff));
                    // in this process of applying polynomial regression in order to derive translation values
                }

                int degree = 2;
                int n = data.Count;
                var X = new DenseMatrix(n, degree + 1);
                var yX = new DenseVector(n);
                var yY = new DenseVector(n);

                for (int i = 0; i < n; i++)
                {
                    double angle = data[i].angle;
                    yX[i] = data[i].point.X;
                    yY[i] = data[i].point.Y;

                    for (int j = 0; j <= degree; j++)
                    {
                        X[i, j] = Math.Pow(angle, j);
                    }
                }

                var Xt = X.Transpose();
                var XtX = Xt * X;
                var XtyX = Xt * yX;
                var XtyY = Xt * yY;

                var coefficientsX = XtX.Solve(XtyX);
                var coefficientsY = XtX.Solve(XtyY);

                return (coefficientsX.ToArray(), coefficientsY.ToArray());
            }
            public UnitVector GetUnitVector()
            {
                UnitVector unitVector;

                List<(float, TShape)> neg = new List<(float, TShape)>();
                List<(float, TShape)> pos = new List<(float, TShape)>();

                foreach (var state in States)
                {
                    if (state.Item1 < 0)
                    {
                        neg.Add(state);
                    }
                    else
                    {
                        pos.Add(state);
                    }
                }

                pos = pos.OrderBy(x => x.Item1).ToList();
                neg = neg.OrderByDescending(x => x.Item1).ToList();


                GetUnitData(pos.ToList(), out Point3f rot, out (double[], double[]) pt);

                GetUnitData(neg.ToList(), out Point3f rot2, out (double[], double[]) nt);

                unitVector = new UnitVector(rot, pt, nt);

                return unitVector;
            }

            void GetUnitData(List<(float, TShape)> shapes, out Point3f uv, out (double[], double[]) tModel)
            {
                TShape last = shapes.Last().Item2;

                PoseTransformation.GetPureRotation(CenterShape, last,
               out Point3f rot);

                Point3f aRot = rot.Abs();

                // max rotation
                float mr = Math.Max(aRot.X, Math.Max(aRot.Y, aRot.Z));

                uv = new Point3f(rot.X / mr, rot.Y / mr, rot.Z / mr);


                // since yaw and pitch dont translate, they dont require a regression model for translation prediction
                tModel = FitModel(shapes);
            }

            public void DrawShapes(Mat frame, Point2f curCenter)
            {
                foreach (var shape in States)
                {
                    Point2f shapeCenter = CenterShape.Centroid;

                    Point2f diff = shape.Item2.Centroid - curCenter;

                    Point2f adjPos = curCenter + diff;

                    Cv2.Circle(frame, adjPos.R2P(), 2, Scalar.Red, 2);
                }
            }
            public void Clear()
            {
                States?.Clear();
            }
        }
    }
}
