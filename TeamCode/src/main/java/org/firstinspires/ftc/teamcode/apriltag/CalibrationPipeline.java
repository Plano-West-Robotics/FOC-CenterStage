package org.firstinspires.ftc.teamcode.apriltag;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CalibrationPipeline extends OpenCvPipeline {
    public static int NEEDED_DATA_POINTS = 10;

    private int state = 0;
    private Mat frameWherePatternWasFound = new Mat();
    private ElapsedTime timeWherePatternWasFound = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Mat inputRGB = new Mat();
    private ArrayList<Mat> objectPoints = new ArrayList<>();
    private ArrayList<Mat> imagePoints = new ArrayList<>();
    private Mat cameraIntrinsicMatrix = new Mat();
    private Mat distortionCoeffs = new Mat();
    private ArrayList<Mat> rvecs = new ArrayList<>();
    private ArrayList<Mat> tvecs = new ArrayList<>();
    private boolean viewportTapped;

    public int dataPointCount = 0;
    public boolean hasEstimateBeenMade = false;
    public double estimateError;
    public AprilTagDetector.LensIntrinsics estimatedIntrinsics;

    /**
     * @param input the frame to be manipulated
     * @return The altered matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        switch (state) {
            case 0: // waiting for viewport tap
                if (this.viewportTapped) {
                    this.viewportTapped = false;
                    state = 1;
                }
                break;
            case 1: // viewport was tapped; looking for calibration pattern
                if (this.viewportTapped) {
                    this.viewportTapped = false;
                }

                Imgproc.cvtColor(input, inputRGB, Imgproc.COLOR_RGBA2RGB);

                FindCalibrationPatternResult res = this.findCircleboard(inputRGB);

                if (res.objectPoints == null) {
                    // found only part of something...

                    Calib3d.drawChessboardCorners(input, res.patternSize, res.imagePoints, false);
                } else {
                    // found something!

                    Mat imagePointsCopy = new Mat();
                    res.imagePoints.copyTo(imagePointsCopy);
                    imagePoints.add(imagePointsCopy);

                    objectPoints.add(res.objectPoints);

                    // updates the public fields; might run `calibrateCamera` if needed
                    this.findLensIntrinsicsIfEnoughData(input);

                    Calib3d.drawChessboardCorners(input, res.patternSize, res.imagePoints, true);
                    input.copyTo(frameWherePatternWasFound);
                    timeWherePatternWasFound.reset();

                    state = 2;
                }

                break;
            case 2: // something was found, now we show the frame where it was found to the user for one second
                if (this.viewportTapped) {
                    this.viewportTapped = false;
                    state = 1;
                }

                if (timeWherePatternWasFound.milliseconds() > 1000) {
                    state = 0;
                }

                frameWherePatternWasFound.copyTo(input);

                break;
        }

        return input;
    }

    @Override
    public void onViewportTapped() {
        this.viewportTapped = true;
    }

    private void findLensIntrinsicsIfEnoughData(Mat input) {
        this.dataPointCount = imagePoints.size();

        if (imagePoints.size() >= NEEDED_DATA_POINTS) {
            rvecs.forEach(Mat::release);
            tvecs.forEach(Mat::release);
            rvecs.clear();
            tvecs.clear();
            double error = Calib3d.calibrateCamera(
                    objectPoints,
                    imagePoints,
                    input.size(),
                    // out params
                    cameraIntrinsicMatrix,
                    distortionCoeffs,
                    rvecs,
                    tvecs
            );
            this.estimateError = error;
            this.estimatedIntrinsics = new AprilTagDetector.LensIntrinsics(
                    cameraIntrinsicMatrix.get(0, 0)[0],
                    cameraIntrinsicMatrix.get(1, 1)[0],
                    cameraIntrinsicMatrix.get(0, 2)[0],
                    cameraIntrinsicMatrix.get(1, 2)[0]
            );
            this.hasEstimateBeenMade = true;

            System.out.println("error: " + this.estimateError);
            System.out.println("fx: " + this.estimatedIntrinsics.fx);
            System.out.println("fy: " + this.estimatedIntrinsics.fy);
            System.out.println("cx: " + this.estimatedIntrinsics.cx);
            System.out.println("cy: " + this.estimatedIntrinsics.cy);
        }
    }

    private static class FindCalibrationPatternResult {
        public MatOfPoint2f imagePoints;
        public MatOfPoint3f objectPoints; // null if nothing was found
        public Size patternSize;
    }

    private MatOfPoint2f circleboardCenters = new MatOfPoint2f();
    private FindCalibrationPatternResult findCircleboard(Mat input) {
        // uses the pattern found here: https://github.com/opencv/opencv/blob/4.x/doc/acircles_pattern.png
        // specifically: asymmetric circles, grid size 11x4

        FindCalibrationPatternResult out = new FindCalibrationPatternResult();

        Size patternSize = new Size(4, 11);

        boolean found = Calib3d.findCirclesGrid(
                input,
                patternSize,
                circleboardCenters,
                Calib3d.CALIB_CB_ASYMMETRIC_GRID
        );

        out.imagePoints = circleboardCenters;
        if (found) {
            out.objectPoints = generatePointsForAcircleboard(patternSize);
        } else {
            out.objectPoints = null;
        }
        out.patternSize = patternSize;

        return out;
    }

    private MatOfPoint2f radonChessboardCorners = new MatOfPoint2f();
    private Mat radonChesssboardMeta = new Mat();
    private FindCalibrationPatternResult findChessboardSB(Mat input) {
        FindCalibrationPatternResult out = new FindCalibrationPatternResult();

        boolean found = Calib3d.findChessboardCornersSBWithMeta(
                input,
                new Size(3, 3),
                radonChessboardCorners,
                Calib3d.CALIB_CB_LARGER,
                radonChesssboardMeta
        );

        out.imagePoints = radonChessboardCorners;
        if (found) {
            out.objectPoints = generatePointsForChessboard(radonChesssboardMeta.size());
        } else {
            out.objectPoints = null;
        }
        out.patternSize = radonChesssboardMeta.size();

        return out;
    }

    private MatOfPoint2f chessboardCorners = new MatOfPoint2f();
    private FindCalibrationPatternResult findChessboard(Mat input) {
        FindCalibrationPatternResult out = new FindCalibrationPatternResult();

        Size patternSize = new Size(9, 14);

        boolean found = Calib3d.findChessboardCorners(
                input,
                patternSize,
                chessboardCorners
        );

        out.imagePoints = chessboardCorners;
        if (found) {
            out.objectPoints = generatePointsForChessboard(patternSize);
        } else {
            out.objectPoints = null;
        }
        out.patternSize = patternSize;

        return out;
    }

    private static MatOfPoint3f generatePointsForChessboard(Size patternSize) {
        int width = (int)patternSize.width;
        int height = (int)patternSize.height;

        MatOfPoint3f out = new MatOfPoint3f(new Mat(1, width * height, CvType.CV_32FC3));
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                out.put(
                        0, row * width + col,
                        new float[] { col, row, 0 }
                );
            }
        }
        return out;
    }

    private static MatOfPoint3f generatePointsForCircleboard(Size patternSize) {
        return generatePointsForChessboard(patternSize);
    }

    private static MatOfPoint3f generatePointsForAcircleboard(Size patternSize) {
        int width = (int)patternSize.width;
        int height = (int)patternSize.height;

        MatOfPoint3f out = new MatOfPoint3f(new Mat(1, width * height, CvType.CV_32FC3));
        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                out.put(
                        0, row * width + col,
                        new float[] { 2 * col + (row % 2), row, 0 }
                );
            }
        }
        return out;
    }

    @Override
    protected void finalize() {
        frameWherePatternWasFound.release();
        inputRGB.release();

        objectPoints.forEach(Mat::release);
        imagePoints.forEach(Mat::release);
        objectPoints.clear();
        imagePoints.clear();

        cameraIntrinsicMatrix.release();
        distortionCoeffs.release();

        rvecs.forEach(Mat::release);
        tvecs.forEach(Mat::release);
        rvecs.clear();
        tvecs.clear();

        circleboardCenters.release();
        radonChessboardCorners.release();
        radonChesssboardMeta.release();
        chessboardCorners.release();
    }
}
