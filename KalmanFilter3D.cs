using OpenCvSharp;
using System;

public class KalmanFilter3D
{
    private KalmanFilter kf;
    private Mat state;  // [x, y, z, vx, vy, vz]
    private Mat meas;

    public KalmanFilter3D()
    {
        kf = new KalmanFilter(6, 3, 0, MatType.CV_32F);
        state = new Mat(6, 1, MatType.CV_32F);
        meas = new Mat(3, 1, MatType.CV_32F);
        state.SetTo(Scalar.All(0));
        meas.SetTo(Scalar.All(0));

        // State transition matrix (assuming constant velocity model)
        kf.TransitionMatrix = new Mat(6, 6, MatType.CV_32F);
        kf.TransitionMatrix.SetArray(new float[] {
            1, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 1,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1
        });

        // Measurement matrix (we only measure position, not velocity)
        kf.MeasurementMatrix = new Mat(3, 6, MatType.CV_32F);
        kf.MeasurementMatrix.SetArray(new float[] {
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0
        });

        float m = 1;
        // Process noise covariance matrix
        Cv2.SetIdentity(kf.ProcessNoiseCov, Scalar.All(.01));

        // Measurement noise covariance matrix
        Cv2.SetIdentity(kf.MeasurementNoiseCov, Scalar.All(.01));

        // Error covariance matrix
        Cv2.SetIdentity(kf.ErrorCovPost, Scalar.All(1));
    }

    public Point3f Predict()
    {
        Mat prediction = kf.Predict();
        return new Point3f(prediction.At<float>(0), prediction.At<float>(1), prediction.At<float>(2));
    }

    public Point3f Update(Point3f measuredPoint)
    {

        meas.Set(0, 0, measuredPoint.X);
        meas.Set(1, 0, measuredPoint.Y);
        meas.Set(2, 0, measuredPoint.Z);

        if (kf.StatePost.Empty())
        {
            kf.StatePost.Set(0, 0, measuredPoint.X);
            kf.StatePost.Set(1, 0, measuredPoint.Y);
            kf.StatePost.Set(2, 0, measuredPoint.Z);
        }

        kf.Correct(meas);
        return Predict();
    }
}

public class SmoothingFilter
{
    private float alpha;
    private Point3f previousValue;

    public SmoothingFilter(float alpha)
    {
        this.alpha = alpha;
        previousValue = new Point3f(0, 0, 0);
    }

    public void ChangeAlpha(float newAlpha)
    {
        alpha = newAlpha;
    }

    public Point3f Smooth(Point3f currentValue)
    {
        Point3f smoothedValue = new Point3f(
            alpha * currentValue.X + (1 - alpha) * previousValue.X,
            alpha * currentValue.Y + (1 - alpha) * previousValue.Y,
            alpha * currentValue.Z + (1 - alpha) * previousValue.Z
        );
        previousValue = smoothedValue;
        return smoothedValue;
    }
}

