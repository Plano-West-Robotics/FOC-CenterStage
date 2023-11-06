package org.firstinspires.ftc.teamcode.apriltag;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagPose;

public class Pose {
//    public static class Rvec {
//        public final MatOfDouble inner;
//
//        public Rvec() {
//            this(0, 0, 0);
//        }
//
//        public Rvec(double x, double y, double z) {
//            this(new MatOfDouble(x, y, z));
//        }
//
//        public Rvec(MatOfDouble inner) {
//            this.inner = inner;
//        }
//
//        public Rvec clone() {
//            return new Rvec((MatOfDouble) this.inner.clone());
//        }
//
//        public void negInPlace() {
//            this.inner.put(0, 0, );
//        }
//
//        @Override
//        protected void finalize() {
//            this.inner.release();
//        }
//    }
//
//    public static class Tvec {
//        public final MatOfDouble inner;
//
//        public Tvec() {
//            this(0, 0, 0);
//        }
//
//        public Tvec(double x, double y, double z) {
//            this(new MatOfDouble(x, y, z));
//        }
//
//        public Tvec(MatOfDouble inner) {
//            this.inner = inner;
//        }
//
//        public Tvec clone() {
//            return new Tvec((MatOfDouble) this.inner.clone());
//        }
//
//        @Override
//        protected void finalize() {
//            this.inner.release();
//        }
//    }

    public final MatOfDouble tvec; // translation vector
    public final MatOfDouble rvec; // rotation vector; direction is axis, magnitude is angle

    public Pose() {
        this(new MatOfDouble(0, 0, 0), new MatOfDouble(0, 0, 0));
    }

    public Pose(
            MatOfDouble tvec,
            MatOfDouble rvec
    ) {
        this.tvec = tvec;
        this.rvec = rvec;
    }

    public void copyFromAprilTagPose(AprilTagPose pose) {
        this.tvec.put(0, 0, pose.x, pose.y, pose.z);

        Mat R = new Mat(3, 3, CvType.CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.put(i, j, pose.R.get(i, j));
            }
        }
        Calib3d.Rodrigues(R, this.rvec);
        R.release();
    }

    public void invert() {
        Core.multiply(this.tvec, new Scalar(-1), this.tvec);
        Core.multiply(this.rvec, new Scalar(-1), this.rvec);

        Mat R = new Mat();
        Calib3d.Rodrigues(this.rvec, R);
        Mat tvec2 = R.matMul(this.tvec);
        tvec2.copyTo(this.tvec);
        tvec2.release();
        R.release();
    }

    /**
     * Modify this <code>Pose</code> in-place to be a pose representing
     * <code>this</code> composed after <code>other</code>.
     * That is, if <code>this</code> represents the pose of object C with
     * respect to object B, and <code>other</code> represents the pose of
     * object B with respect to object A, then after this method is called,
     * <code>this</code> will represent the pose of object C with respect to
     * object A.
     *
     * @param other The other pose to be composed before this one.
     */
    public void composeAfter(Pose other) {
        Mat R1 = new Mat();
        Calib3d.Rodrigues(other.rvec, R1);
        Mat R2 = new Mat();
        Calib3d.Rodrigues(this.rvec, R2);

        Mat tvec2 = R1.matMul(this.tvec);
        Core.add(other.tvec, tvec2, this.tvec);
        tvec2.release();

        Mat R3 = R1.matMul(R2);
        Calib3d.Rodrigues(R3, this.rvec);
        R3.release();

        R1.release();
        R2.release();
    }

    public void release() {
        this.rvec.release();
        this.tvec.release();
    }
}