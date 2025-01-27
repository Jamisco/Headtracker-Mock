using OpenCvSharp;
using ScottPlot;
using ScottPlot.PlotStyles;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;
using static HeadTracker;

namespace Headtracker_Console
{
    [Serializable]
    public class Calibration
    {
        public static string savePath = "calibration.json";
        List<Triangle> midToMin = new List<Triangle>();
        List<Triangle> midToMax = new List<Triangle>();

        [JsonInclude]
        CalibrationFrameHolder pitchHolder;

        [JsonInclude]
        CalibrationFrameHolder yawHolder;

        [JsonInclude]
        CalibrationFrameHolder rollHolder;

        public TOffsets PitchOffset;
        public TOffsets YawOffset;
        public TOffsets RollOffset;


        private List<Triangle> pitchPoints = new List<Triangle>();
        private List<Triangle> yawPoints = new List<Triangle>();
        private List<Triangle> rollPoints = new List<Triangle>();


        HeadTracker headTracker;
        public bool InitCalibrate { get; private set; } = false;
        public bool IsCalibrating { get; set; } = false;

        public bool CalibratePitch { get; set; } = false;
        public bool CalibrateYaw { get; set; } = false;
        public bool CalibrateRoll { get; set; } = false;

        public bool PitchCalibrated
        {
            get { return PitchOffset.maxLength > 0; }
        }
        public bool YawCalibrated
        {
            get { return YawOffset.maxLength > 0; }
        }

        public bool RollCalibrated
        {
            get { return RollOffset.maxLength > 0; }
        }

        public bool FullyCalibrated
        {
            get { return PitchCalibrated && YawCalibrated && RollCalibrated; }
        }

        public Calibration(HeadTracker ht)
        {
            Deserialize();
            headTracker = ht;
        }
        public void Serialize()
        {
            string test = JsonSerializer.Serialize(this, options);

            File.WriteAllText(savePath, test);
        }

        public void Deserialize()
        {
            if (!File.Exists(savePath))
            {
                YawOffset = new TOffsets();
                PitchOffset = new TOffsets();
                RollOffset = new TOffsets();
            }
            else
            {
                string text = File.ReadAllText(savePath);

                try
                {
                    Calibration c = JsonSerializer.Deserialize<Calibration>(text, options);

                    pitchHolder = c.pitchHolder;
                    yawHolder = c.yawHolder;
                    rollHolder = c.rollHolder;

                    CalibrateOffsets();
                }
                catch (Exception ex)
                {
                    //Attempt to recover individual properties
                    try
                    {
                        var test = GetProperty(text, "pitchHolder");
                        this.pitchHolder = CalibrationFrameHolder.Deserialize(test);

                        var test2 = GetProperty(text, "yawHolder");
                        this.yawHolder = CalibrationFrameHolder.Deserialize(test2);

                        var test3 = GetProperty(text, "rollHolder");
                        this.rollHolder = CalibrationFrameHolder.Deserialize(test3);

                        CalibrateOffsets();

                        InitCalibrate = true;
                    }
                    catch (Exception)
                    {
                        int he = 2;
                        //pitchHolder = new CalibrationFrameHolder();
                        //yawHolder = new CalibrationFrameHolder();
                        //rollHolder = new CalibrationFrameHolder();

                        //YawOffset = new TOffsets();
                        //PitchOffset = new TOffsets();
                        //RollOffset = new TOffsets();

                        //File.WriteAllText(savePath, "");
                    }
                }
            }
        }

        private static string GetProperty(string json, string propertyName)
        {
            using (JsonDocument document = JsonDocument.Parse(json))
            {
                if (document.RootElement.TryGetProperty(propertyName, out JsonElement property))
                {
                    return property.GetRawText();
                }
            }
            return null;
        }
        public void SetYawOffset(Triangle center, Triangle minPoint, Triangle maxPoint)
        {
            YawOffset = new TOffsets(yawHolder);
        }
        public void CalibrateOffsets()
        {
            if (pitchHolder.CanCalibrate)
            {
                PitchOffset = new TOffsets(pitchHolder);
            }

            if (yawHolder.CanCalibrate)
            {
                YawOffset = new TOffsets(yawHolder);
            }

            if (rollHolder.CanCalibrate)
            {
                RollOffset = new TOffsets(rollHolder);
            }
        }
        public void GetTransformation(Triangle center, Triangle t1, out Triangle t2, out float yaw, out float pitch)
        {
            yaw = 1f;

            PitchOffset.OffsetToCenter(center, t1, out t2, out pitch);
            YawOffset.OffsetToCenter(center, t2, out t2, out yaw);
            RollOffset.OffsetToCenter(center, t2, out t2, out float roll);
        }

        public void GetTransformation2(Triangle center, Triangle t1, out Triangle t2)
        {
            int count = 5;
            t2 = t1;

            for(int i = 0; i < count; i++)
            {
                //PitchOffset.OffsetToCenter2(center, t2, out t2);
                YawOffset.OffsetToCenter2(center, t2, out t2);
                //RollOffset.OffsetToCenter2(center, t2, out t2);
            }
        }

        public void TestAngle(Triangle ct, out Triangle t2)
        {
            PitchOffset.TestAngleOffset(ct, out t2);
        }

        static JsonSerializerOptions options = new JsonSerializerOptions
        {
            IncludeFields = true,
            IgnoreReadOnlyProperties = true,
            WriteIndented = true
        };

        public void UpdateCenter()
        {
            pitchHolder.UpdateCenter(headTracker.centerTriangle);
            yawHolder.UpdateCenter(headTracker.centerTriangle);
            rollHolder.UpdateCenter(headTracker.centerTriangle);
        }
        public void AddCalibrationPoint(ref Mat displayFrame)
        {
            if (IsCalibrating)
            {
                if (InitCalibrate == false)
                {
                    pitchHolder = new CalibrationFrameHolder(headTracker.centerTriangle, Axis.YAxis);

                    yawHolder = new CalibrationFrameHolder(headTracker.centerTriangle, Axis.XAxis);

                    rollHolder = new CalibrationFrameHolder(headTracker.centerTriangle, Axis.XAxis);

                    InitCalibrate = true;
                }

                Triangle c = headTracker.curTriangle;

                if(!c.IsValid)
                {
                    return;
                }

                if (CalibratePitch)
                {
                    pitchHolder.AddTriangle(c);
                    pitchHolder.ShowPoints(ref displayFrame);
                }

                if (CalibrateYaw)
                {
                    yawHolder.AddTriangle(c);
                    yawHolder.ShowPoints(ref displayFrame);
                }

                if (CalibrateRoll)
                {
                    rollHolder.AddTriangle(c);
                    rollHolder.ShowPoints(ref displayFrame);
                }
            }
        }

        public void ClearCurrentPoints()
        {
            if (CalibratePitch && InitCalibrate)
            {
                pitchHolder.ClearPoints();
            }

            if (CalibrateYaw && InitCalibrate)
            {
                yawHolder.ClearPoints();
            }

            if (CalibrateRoll && InitCalibrate)
            {
                rollHolder.ClearPoints();
            }
        }

        [Serializable]
        public struct TOffsets
        {
            public Axis axis;

            public TriangleDiff maxUnitOffset;
            public TriangleDiff minUnitOffset;

            public float minLength;
            public float maxLength;

            public TriangleDiff maxAngleUnitOffset;
            public TriangleDiff minAngleUnitOffset;

            public float minAngle;
            public float maxAngle;

            public int angleIndex;

            Triangle center;

            private void Init()
            {
                // just to copy

                minUnitOffset = new TriangleDiff();
                maxUnitOffset = new TriangleDiff();

                minLength = 0;
                maxLength = 0;

                minAngleUnitOffset = new TriangleDiff();
                maxAngleUnitOffset = new TriangleDiff();

                minAngle = 0;
                maxAngle = 0;

                angleIndex = 0;
            }

            public TOffsets(CalibrationFrameHolder frames)
            {
                minUnitOffset = new TriangleDiff();
                maxUnitOffset = new TriangleDiff();

                minLength = 0;
                maxLength = 0;

                minAngleUnitOffset = new TriangleDiff();
                maxAngleUnitOffset = new TriangleDiff();

                minAngle = 0;
                maxAngle = 0;

                angleIndex = 0;

                axis = frames.axis;

                center = frames.CenterTriangle;

                minUnitOffset = GetUnitOffset(frames.MidToMin, out minLength);
                maxUnitOffset = GetUnitOffset(frames.MidToMax, out maxLength);

                int minAngleIndex = 0;
                int maxAngleIndex = 0;

                minAngleUnitOffset = GetMaxAngleOffset(frames.MidToMin, out minAngle, out minAngleIndex);

                maxAngleUnitOffset = GetMaxAngleOffset(frames.MidToMax, out maxAngle, out maxAngleIndex);

                angleIndex = (maxLength > minLength) ? maxAngleIndex : minAngleIndex;
            }

            private TriangleDiff GetUnitOffset(List<Triangle> frames, out float length)
            {
                TriangleDiff unitOffset = new TriangleDiff();

                List<Point2f> centriods = new List<Point2f>();

                float oppositeLength;

                foreach (Triangle t in frames)
                {
                    centriods.Add(t.Centroid);
                }

                // Get the point2f which is the middle of all points
                int average = 0;
                Point2f closest;

                if (axis == Axis.XAxis)
                {
                    oppositeLength = Math.Abs(frames.Last().Centroid.Y
                     - frames.First().Centroid.Y);

                    average = (int)centriods.Average(x => x.X);
                    closest = centriods.OrderBy(x => Math.Abs(x.X - average)).First();

                    Triangle min = frames[0];
                    Triangle max = frames.First(x => x.Centroid == closest);

                    unitOffset = new TriangleDiff(min, max);

                    unitOffset /= unitOffset.centriodDiff.X;

                    length = Math.Abs(frames.Last().Centroid.X
                                            - frames.First().Centroid.X);
                }
                else
                {
                    oppositeLength = Math.Abs(frames.Last().Centroid.X
                                         - frames.First().Centroid.X);

                    average = (int)centriods.Average(x => x.Y);
                    closest = centriods.OrderBy(x => Math.Abs(x.Y - average)).First();

                    Triangle min = frames[0];
                    Triangle max = frames.First(x => x.Centroid == closest);

                    unitOffset = new TriangleDiff(min, max);

                    unitOffset /= unitOffset.centriodDiff.Y;

                    unitOffset.centriodDiff = unitOffset.centriodDiff * -1;

                    length = Math.Abs(frames.Last().Centroid.Y
                                            - frames.First().Centroid.Y);
                }

                // now we have a point that lies on the line of best fit, calculate the unit offset

                length = Math.Abs(length);
                unitOffset.centriodDiff = unitOffset.centriodDiff.Abs();
                return unitOffset;
            }


            // will find which side changed the most and use that as the reference for the unitoffset. This differs from the original where as in the original method simply offset the triangle by the centriod, this one doesnt. 
            private TriangleDiff GetMaxAngleOffset(List<Triangle> frames, out float length, out int index)
            {
                TriangleDiff diff = new TriangleDiff(frames[0], frames.Last());

                float maxAngleDiff = 0;
                int maxAngleIndex = 0;

                for (int i = 0; i < 3; i++)
                {
                    float ta = Math.Abs(frames[0].Angles[i] - frames.Last().Angles[i]);

                    if (ta > maxAngleDiff)
                    {
                        maxAngleDiff = ta;
                        maxAngleIndex = i;
                    }
                }

                length = frames.Last().Angles[maxAngleIndex];
                index = maxAngleIndex;

                // with this, we know that for the side of the triangle that had the max angle difference, we can now increase/decrease a triangle by 1 angle unit
                return diff / maxAngleDiff;
            }
            public void OffsetToCenter(Triangle center, Triangle t1, out Triangle t2, out float percent)
            {
                TriangleDiff diff = new TriangleDiff(center, t1);
                TriangleDiff unitOffset = new TriangleDiff();

                // we do this to see whether to negate the offset.
                // this is because if we want to move the point to the center, we need to know which direction to move it.

                int xNeg = (diff.centriodDiff.X > 0) ? -1 : 1;
                int yNeg = (diff.centriodDiff.Y > 0) ? -1 : 1;

                // max unit multiplier
                float mam = 0;

                float length = 0;

                if (axis == Axis.YAxis)
                {
                    unitOffset = (yNeg == 1) ? minUnitOffset : maxUnitOffset;
                    length = (yNeg == 1) ? minLength : maxLength;

                    mam = Math.Abs(diff.centriodDiff.Y / unitOffset.centriodDiff.Y);
                }
                else
                {
                    unitOffset = (xNeg == 1) ? maxUnitOffset : minUnitOffset;
                    length = (xNeg == 1) ? maxLength : minLength;
                    mam = Math.Abs(diff.centriodDiff.X / unitOffset.centriodDiff.X);
                }

                percent = mam / length;

                TriangleDiff dOff = (unitOffset * mam);

                for (int i = 0; i < dOff.vectorDiff.Length; i++)
                {
                    Point2f v = dOff.vectorDiff[i];

                    // for some movements, the sizes of the triangle may increase then decrease or vice versa. Thus we cannot have only 1 unit offset for the entire movement.

                    // also when readjusting, your are accounting only for the centriod diffs, not the triangle lengths,

                    // you need to measure the length, see which one can be increased or decreased the least while still maintaining thesame ratio of the triangle as the original.

                    // Going off the centriod alone is stupid

                    float dl = diff.vectorDiff[i].Magnitude();
                    float du = dOff.vectorDiff[i].Magnitude();

                    //if (du > dl)
                    //{
                    //    dOff.vectorDiff[i] = dOff.vectorDiff[i] * 1;
                    //}
                    //else
                    //{
                    //    dOff.vectorDiff[i] = dOff.vectorDiff[i] * -1;
                    //}
                }

                Point2f c = dOff.centriodDiff;

                c.X *= xNeg;
                c.Y *= yNeg;

                dOff.centriodDiff = c;

                t2 = t1 + dOff;

            }

            public void OffsetToCenter2(Triangle center, Triangle t1, out Triangle t2)
            {
                int side = 0;
                float angle = 0;

                if(axis == Axis.XAxis)
                {
                    side = t1.Centroid.X > center.Centroid.X ? 1 : -1;
                }
                else
                {
                    side = t1.Centroid.Y > center.Centroid.Y ? 1 : -1;
                }

                angle = center.Angles[angleIndex];

                float t1Angle = t1.Angles[angleIndex];

                float unit = Math.Abs(angle - t1Angle);

                OffsetAngleByUnit(t1, unit, side, out t2);
            }
            public void OffsetAngleByUnit(Triangle t1, float unit, int side, out Triangle t2)
            {
                TriangleDiff angleOffset = new TriangleDiff();
                float centerAngle = t1.Angles[angleIndex];

                if (side == 1)
                {
                    angleOffset = maxAngleUnitOffset;
                }
                else
                {
                    angleOffset = minAngleUnitOffset;
                }

                t2 = t1 + (angleOffset * unit);

                //t2.TranslateTo(center);
            }


            private static float angle = 1;
            private static int dir = -1;
            private static int limit = 20;
            private static int side = 1;
            public void TestAngleOffset(Triangle ct, out Triangle offset)
            {
                // roll angles to min and max angles
                // do min then max
                
                if(angle == limit)
                {
                    dir *= -1;
                }
                else if (angle == 0)
                {
                    side *= -1;
                    dir *= -1;
                }

                angle += dir;

                Triangle t2 = new Triangle();

                OffsetAngleByUnit(ct, angle, side, out offset);
            }
        }

        [Serializable]
        public struct CalibrationFrameHolder
        {
            public List<Triangle> MidToMin;
            public List<Triangle> MidToMax;
            public Axis axis;

            public float midToMinLength;
            public float midToMaxLength;

            [JsonInclude]
            public Triangle centerTriangle;

            public Triangle CenterTriangle
            {
                get { return centerTriangle; }
            }

            public bool CanCalibrate
            {
                get { return MidToMin?.Count > 0 && MidToMax?.Count > 0; }
            }


            public CalibrationFrameHolder(Triangle centerTriangle, Axis axis)
            {
                this.centerTriangle = centerTriangle;
                MidToMin = new List<Triangle>();
                MidToMax = new List<Triangle>();

                this.axis = axis;

                midToMinLength = 0;
                midToMaxLength = 0;
            }

            public static CalibrationFrameHolder Deserialize(string text)
            {
                CalibrationFrameHolder f1 = JsonSerializer.Deserialize<CalibrationFrameHolder>(text, options);

                // check if any object is null

                f1.MidToMin = f1.MidToMin ?? new List<Triangle>();
                f1.MidToMax = f1.MidToMax ?? new List<Triangle>();

                return f1;

            }

            public void UpdateCenter(Triangle center)
            {
                // all the points are relative to the center, so we need to offset them by the new center

                if(!centerTriangle.IsValid || !center.IsValid)
                {
                    centerTriangle = center;

                    return;
                }

                Point2f offset = centerTriangle.Centroid - center.Centroid;

                for (int i = 0; i < MidToMin.Count; i++)
                {
                    Triangle point = MidToMin[i];
                    point.Translate(offset);

                    MidToMin[i] = point;
                }

                for (int i = 0; i < MidToMax.Count; i++)
                {
                    Triangle point = MidToMax[i];
                    point.Translate(offset);

                    MidToMax[i] = point;
                }

                centerTriangle = center;

            }

            public void AddTriangle(Triangle triangle)
            {
                if (axis == Axis.YAxis)
                {
                    bool toMax = triangle.Centroid.Y > centerTriangle.Centroid.Y;

                    if (toMax)
                    {
                        if (MidToMax.Count == 0)
                        {
                            MidToMax.Add(triangle);
                        }
                        else
                        {
                            Triangle last = MidToMax.Last();

                            if (triangle.Centroid.Y > last.Centroid.Y)
                            {
                                MidToMax.Add(triangle);
                            }
                        }
                    }
                    else
                    {
                        if (MidToMin.Count == 0)
                        {
                            MidToMin.Add(triangle);
                        }
                        else
                        {
                            Triangle last = MidToMin.Last();

                            if (triangle.Centroid.Y < last.Centroid.Y)
                            {
                                MidToMin.Add(triangle);
                            }
                        }
                    }
                }
                else
                {
                    bool toMax = triangle.Centroid.X > centerTriangle.Centroid.X;

                    if (toMax)
                    {
                        if (MidToMax.Count == 0)
                        {
                            MidToMax.Add(triangle);
                        }
                        else
                        {
                            Triangle last = MidToMax.Last();

                            if (triangle.Centroid.X > last.Centroid.X)
                            {
                                MidToMax.Add(triangle);
                            }
                        }
                    }
                    else
                    {
                        if (MidToMin.Count == 0)
                        {
                            MidToMin.Add(triangle);
                        }
                        else
                        {
                            Triangle last = MidToMin.Last();

                            if (triangle.Centroid.X < last.Centroid.X)
                            {
                                MidToMin.Add(triangle);
                            }
                        }
                    }
                }
            }

            public void ClearPoints()
            {
                MidToMax.Clear();
                MidToMin.Clear();
            }
            public void PlotPoints(string name)
            {
                Plot plot = new Plot();

                double[] x1 = MidToMin.Select(x => (double)x.Centroid.X).ToArray();
                double[] y1 = MidToMin.Select(x => (double)x.Centroid.Y).ToArray();

                double[] x2 = MidToMax.Select(x => (double)x.Centroid.X).ToArray();
                double[] y2 = MidToMax.Select(x => (double)x.Centroid.Y).ToArray();

                plot.Add.Scatter(x1, y1, ScottPlot.Color.FromColor(System.Drawing.Color.Green));
                plot.Add.Scatter(x2, y2, ScottPlot.Color.FromColor(System.Drawing.Color.Red));


                plot.SavePng(name + ".png", 500, 500);

                // create seperate gui track to slowly follow the line
            }
            public void ShowPoints(ref Mat emptyFrame)
            {
                int ra = 2;

                foreach (var point in MidToMin)
                {
                    emptyFrame.Circle(point.Centroid.R2P(), ra, Scalar.Green, -1);
                }

                foreach (var point in MidToMax)
                {
                    emptyFrame.Circle(point.Centroid.R2P(), ra, Scalar.Red, -1);
                }
            }
            int GetDirection(Point2f center, Point2f curCentroid, Axis axis)
            {
                if (axis == Axis.XAxis)
                {
                    return curCentroid.X > center.X ? 1 : -1;
                }
                else
                {
                    return curCentroid.Y > center.Y ? 1 : -1;
                }
            }
        }
    }
}
