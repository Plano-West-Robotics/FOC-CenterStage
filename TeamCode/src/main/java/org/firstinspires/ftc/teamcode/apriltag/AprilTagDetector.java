package org.firstinspires.ftc.teamcode.apriltag;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.util.Iterator;

public class AprilTagDetector {
    // android studio believes this initializer is unneeded, but i think `createApriltagDetector` can throw an exception so it actually is
    private long nativePtr = 0;

    public static class LensIntrinsics {
        public final double fx;
        public final double fy;
        public final double cx;
        public final double cy;

        // circleboard
        // error : 0.18582141182785794
        // fx : 628.3347909909166
        // fy : 627.7113974398933
        // cx : 349.504375726049
        // cy : 218.421352607656

        public static final LensIntrinsics FOR_ALURATEK_1080P_CAMERA_AT_640X480 = new LensIntrinsics(
                628.335,
                627.711,
                349.504,
                218.421
        );
        public static final LensIntrinsics FOR_FACETIME_HD_CAMERA_AT_640X480 = new LensIntrinsics(
                643.130,
                642.344,
                324.831,
                246.725
        );
        public static final LensIntrinsics FOR_C920_CAMERA_AT_800X448 = new LensIntrinsics(
                578.272,
                578.272,
                402.145,
                221.506
        );
        public static final LensIntrinsics FOR_C270_CAMERA_AT_800X448 = new LensIntrinsics(
                857.212,
                853.720,
                405.908,
                215.425
        );

        public LensIntrinsics(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
        }
    }

    public AprilTagDetector(AprilTagDetectorJNI.TagFamily family, float decimation, int threads) {
        this.nativePtr = AprilTagDetectorJNI.createApriltagDetector(family.string, decimation, threads);
    }

    private void checkNativePtrNonnull() {
        if (this.nativePtr == 0) {
            throw new RuntimeException("AprilTagDetector.nativePtr was NULL");
        }
    }

    @Override
    protected void finalize() {
        if (this.nativePtr != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(this.nativePtr);
        }
    }

    public void setDecimation(float decimation) {
        this.checkNativePtrNonnull();

        AprilTagDetectorJNI.setApriltagDetectorDecimation(this.nativePtr, decimation);
    }

    /**
     * Run the tag detection algorithm on the provided grayscale image.
     *
     * @param mat The OpenCV matrix representing the grayscale image.
     * @param intrinsics The intrinsics of the lens used to record the image.
     * @return An iterable over all the detections
     */
    public AprilTagDetections run(Mat mat, LensIntrinsics intrinsics) {
        this.checkNativePtrNonnull();

        long detectionsPtr = AprilTagDetectorJNI.runApriltagDetector(this.nativePtr, mat.dataAddr(), mat.width(), mat.height());
        return new AprilTagDetections(detectionsPtr, intrinsics);
    }

    public static class AprilTagDetection {
        // this backreference prevents the `AprilTagDetections` class from being finalized and freeing `this.detectionPtr`
        private final AprilTagDetections origin;

        public final int id;
        public final int hamming;
        public final float decisionMargin;
        public final Point center;
        public final Point[] corners;
        private final long detectionPtr; // used to obtain the pose estimate once the caller determines the tag size

        private AprilTagDetection(
                AprilTagDetections origin,
                int id,
                int hamming,
                float decisionMargin,
                Point center,
                Point[] corners,
                long detectionPtr
        ) {
            this.origin = origin;
            this.id = id;
            this.hamming = hamming;
            this.decisionMargin = decisionMargin;
            this.center = center;
            this.corners = corners;
            this.detectionPtr = detectionPtr;
        }

        /**
         * Get an estimate for the pose of the tag. This is a seperate method to
         * allow determining the tag size based off of the id.
         *
         * @param tagSize The size of the tag, in metres.
         * @return The estimate of the tag's pose.
         */
        public AprilTagPose getPoseEstimate(double tagSize) {
            LensIntrinsics intrinsics = this.origin.intrinsics;
            double[] poseEstimate = ApriltagDetectionJNI.getPoseEstimate(
                    this.detectionPtr,
                    tagSize,
                    intrinsics.fx,
                    intrinsics.fy,
                    intrinsics.cx,
                    intrinsics.cy
            );

            AprilTagPose out = new AprilTagPose();

            out.x = poseEstimate[0];
            out.y = poseEstimate[1];
            out.z = poseEstimate[2];

            float[] rotMtxVals = new float[3*3];
            for (int i = 0; i < 9; i++) {
                rotMtxVals[i] = (float)poseEstimate[3 + i];
            }
            out.R = new GeneralMatrixF(3, 3, rotMtxVals);

            return out;
        }
    }

    public static class AprilTagDetections implements Iterable<AprilTagDetection> {
        private final long detectionsPtr; // needed to free the memory
        private final long[] detectionPtrs;
        private final LensIntrinsics intrinsics; // needed to get the pose estimate

        private AprilTagDetections(long detectionsPtr, LensIntrinsics intrinsics) {
            this.detectionsPtr = detectionsPtr; // may be zero if nothing was detected
            if (detectionsPtr != 0) {
                this.detectionPtrs = ApriltagDetectionJNI.getDetectionPointers(detectionsPtr);
            } else {
                this.detectionPtrs = new long[0];
            }
            this.intrinsics = intrinsics;
        }

        @Override
        protected void finalize() {
            if (this.detectionsPtr != 0) {
                ApriltagDetectionJNI.freeDetectionList(this.detectionsPtr);
            }
        }

        @NonNull
        @Override
        public Iterator<AprilTagDetection> iterator() {
            return new AprilTagDetectionIterator(this);
        }
    }

    public static class AprilTagDetectionIterator implements Iterator<AprilTagDetection> {
        // this backreference prevents the `AprilTagDetections` class from being finalized and freeing `this.detectionPtrs`
        private final AprilTagDetections origin;

        private final long[] detectionPtrs;
        private int index;

        private AprilTagDetectionIterator(AprilTagDetections origin) {
            this.origin = origin;
            this.detectionPtrs = origin.detectionPtrs;
            this.index = 0;
        }

        @Override
        public boolean hasNext() {
            return this.index < this.detectionPtrs.length;
        }

        @Override
        public AprilTagDetection next() {
            if (!this.hasNext()) {
                return null;
            }

            long detectionPtr = this.detectionPtrs[this.index++];

            double[] centerRaw = ApriltagDetectionJNI.getCenterpoint(detectionPtr);
            Point center = new Point(centerRaw[0], centerRaw[1]);
            double[][] cornersRaw = ApriltagDetectionJNI.getCorners(detectionPtr);
            Point[] corners = new Point[4];
            for (int i = 0; i < 4; i++) {
                corners[i] = new Point(cornersRaw[i][0], cornersRaw[i][1]);
            }

            AprilTagDetection detection = new AprilTagDetection(
                    this.origin,
                    ApriltagDetectionJNI.getId(detectionPtr),
                    ApriltagDetectionJNI.getHamming(detectionPtr),
                    ApriltagDetectionJNI.getDecisionMargin(detectionPtr),
                    center,
                    corners,
                    detectionPtr
            );

            return detection;
        }
    }
}
