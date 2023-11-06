package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class AprilTagPipeline extends OpenCvPipeline {
    private AprilTagDetector detector;

    private Mat gray = new Mat();

    private final AprilTagDetector.LensIntrinsics intrinsics = AprilTagDetector.LensIntrinsics.FOR_FACETIME_HD_CAMERA_AT_640X480;
    private final Mat cameraMatrix;

    private static class TagLocation {
        public double size;
        public Pose pose;

        public TagLocation(double size, Pose pose) {
            this.size = size;
            this.pose = pose;
        }
    }

    private static final double TWO_PI_OVER_THREE_ROOT_THREE = Math.sqrt(4 * Math.PI * Math.PI / 27);
    private static final double MM_PER_INCH = 25.4;
    private static final Pose DUMMY_POSE = new Pose(new MatOfDouble(), new MatOfDouble()); // TODO: actually put poses of every tag
    private static final TagLocation[] TAGS = new TagLocation[] {
            // Blue Alliance Backdrop (1 = left, 2 = center, 3 = right)
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),

            // Red Alliance Backdrop (4 = left, 5 = center, 6 = right)
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),

            // Red Alliance Audience Wall (Tile E1)
            new TagLocation(5 * MM_PER_INCH, new Pose(
                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, -41 * MM_PER_INCH, 5.5 * MM_PER_INCH),
                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
            )),
            new TagLocation(2 * MM_PER_INCH, new Pose(
                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, -35.5 * MM_PER_INCH, 4 * MM_PER_INCH),
                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
            )),

            // Blue Alliance Audience Wall (Tile B1)
            new TagLocation(2 * MM_PER_INCH, new Pose(
                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 35.5 * MM_PER_INCH, 4 * MM_PER_INCH),
                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
            )),
            new TagLocation(5 * MM_PER_INCH, new Pose(
                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 41 * MM_PER_INCH, 5.5 * MM_PER_INCH),
                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
            )),
    };

    public static final TagGroup[] TAG_GROUPS = new TagGroup[] {
            new TagGroup(
                    new Pose(
                            new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 35.5 * MM_PER_INCH, 4 * MM_PER_INCH),
                            new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
                    ),
                    new TagGroup.Tag(9, new Point(0, 0), 2 * MM_PER_INCH),
                    new TagGroup.Tag(10, new Point(5.5 * MM_PER_INCH, -1.5 * MM_PER_INCH), 5 * MM_PER_INCH)
            )
    };

    public Pose[] cameraPoseEstimates = new Pose[TAG_GROUPS.length];
    public boolean[] isVisible = new boolean[TAG_GROUPS.length];

    public AprilTagPipeline() {
        this.detector = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);

        this.cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        this.cameraMatrix.put(0, 0, this.intrinsics.fx);
        this.cameraMatrix.put(0, 1, 0);
        this.cameraMatrix.put(0, 2, this.intrinsics.cx);

        this.cameraMatrix.put(1, 0, 0);
        this.cameraMatrix.put(1, 1, this.intrinsics.fy);
        this.cameraMatrix.put(1, 2, this.intrinsics.cy);

        this.cameraMatrix.put(2, 0, 0);
        this.cameraMatrix.put(2, 1, 0);
        this.cameraMatrix.put(2 ,2, 1);

        for (int i = 0; i < cameraPoseEstimates.length; i++) {
            cameraPoseEstimates[i] = new Pose();
        }
    }

    /**
     * @param input the frame to be manipulated
     * @return The altered matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
        AprilTagDetector.AprilTagDetections detections = this.detector.run(gray, this.intrinsics);

        AprilTagDetector.AprilTagDetection[] detectionsArr = new AprilTagDetector.AprilTagDetection[10];
        for (AprilTagDetector.AprilTagDetection detection : detections) {
            int i = detection.id - 1;

            if (i >= TAGS.length) {
                continue;
            }

            detectionsArr[i] = detection;
        }

        boolean[] localIsVisible = new boolean[isVisible.length];

        for (int i = 0; i < TAG_GROUPS.length; i++) {
            TagGroup group = TAG_GROUPS[i];

            ArrayList<Point3> objectPoints = new ArrayList<>();
            ArrayList<Point> imagePoints = new ArrayList<>();

            boolean anyTagsFound = false;
            for (TagGroup.Tag tag : group.tags) {
                int j = tag.id - 1;
                if (detectionsArr[j] == null) {
                    continue;
                }
                AprilTagDetector.AprilTagDetection detection = detectionsArr[j];

                anyTagsFound = true;

                objectPoints.add(new Point3(
                        tag.pos.x,
                        tag.pos.y,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x - tag.size/2,
                        tag.pos.y + tag.size/2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x + tag.size/2,
                        tag.pos.y + tag.size/2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x + tag.size/2,
                        tag.pos.y - tag.size/2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x - tag.size/2,
                        tag.pos.y - tag.size/2,
                        0
                ));

                imagePoints.add(detection.center);
                imagePoints.add(detection.corners[0]);
                imagePoints.add(detection.corners[1]);
                imagePoints.add(detection.corners[2]);
                imagePoints.add(detection.corners[3]);
            }

            if (!anyTagsFound) {
                continue;
            }

            MatOfPoint3f objectPointsMat = new MatOfPoint3f();
            objectPointsMat.fromList(objectPoints);
            MatOfPoint2f imagePointsMat = new MatOfPoint2f();
            imagePointsMat.fromList(imagePoints);

            Pose pose = cameraPoseEstimates[i];

            Calib3d.solvePnP(
                    objectPointsMat,
                    imagePointsMat,
                    cameraMatrix,
                    new MatOfDouble(),
                    pose.rvec,
                    pose.tvec,
                    false,
                    Calib3d.SOLVEPNP_IPPE
            );

            pose.invert();
            pose.composeAfter(group.pose);

            localIsVisible[i] = true;
        }

//        for (AprilTagDetector.AprilTagDetection detection : detections) {
//            if (detection.id > TAGS.length) {
//                continue;
//            }
//            TagLocation tagLocation = TAGS[detection.id - 1];
//            double tagSize = tagLocation.size;
//
//            AprilTagPose aprilTagPose = detection.getPoseEstimate(tagSize);
//            Pose pose = cameraPoseBasedOffTag[detection.id - 1];
//            pose.copyFromAprilTagPose(aprilTagPose);
//            localIsTagVisible[detection.id - 1] = true;
//
//            MatOfPoint3f tagObjectPoints = new MatOfPoint3f(
//                    new Point3(-tagSize / 2, tagSize / 2, 0),
//                    new Point3(tagSize / 2, tagSize / 2, 0),
//                    new Point3(tagSize / 2, -tagSize / 2, 0),
//                    new Point3(-tagSize / 2, -tagSize / 2, 0)
////                    new Point3(0, 0, 0)
//            );
//
//            MatOfPoint2f tagImagePoints = new MatOfPoint2f(
//                    detection.corners[0],
//                    detection.corners[1],
//                    detection.corners[2],
//                    detection.corners[3]
////                    detection.center
//            );
//
//            Calib3d.solvePnP(
//                    tagObjectPoints,
//                    tagImagePoints,
//                    cameraMatrix,
//                    new MatOfDouble(),
//                    pose.rvec,
//                    pose.tvec,
//                    true,
//                    Calib3d.SOLVEPNP_ITERATIVE
//            );
//
//            this.drawTag(input, tagLocation.size, detection, pose);
//
//            pose.invert();
//            pose.composeAfter(tagLocation.pose);
//        }

        System.arraycopy(localIsVisible, 0, isVisible, 0, isVisible.length);

        return input;
    }

    private void drawTag(Mat input, double tagSize, AprilTagDetector.AprilTagDetection detection, Pose pose) {
        double axesLen = tagSize / 4;
        double boxSize = tagSize * 5 / 4;
        MatOfPoint3f pointsToProject = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axesLen, 0, 0),
                new Point3(0, axesLen, 0),
                new Point3(0, 0, -axesLen),

                new Point3(boxSize / 2, boxSize / 2, 0),
                new Point3(-boxSize / 2, boxSize / 2, 0),
                new Point3(boxSize / 2, -boxSize / 2, 0),
                new Point3(-boxSize / 2, -boxSize / 2, 0),
                new Point3(boxSize / 2, boxSize / 2, -boxSize),
                new Point3(-boxSize / 2, boxSize / 2, -boxSize),
                new Point3(boxSize / 2, -boxSize / 2, -boxSize),
                new Point3(-boxSize / 2, -boxSize / 2, -boxSize)
        );

        MatOfPoint2f projectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(pointsToProject, pose.rvec, pose.tvec, this.cameraMatrix, new MatOfDouble(), projectedPoints);
        Point[] points = projectedPoints.toArray();

        int thickness = 1;
        Scalar xcol = new Scalar(255, 0, 0);
        Scalar ycol = new Scalar(0, 255, 0);
        Scalar zcol = new Scalar(0, 0, 255);

        // axes
        Imgproc.line(input, points[0], points[1], xcol, thickness);
        Imgproc.line(input, points[0], points[2], ycol, thickness);
        Imgproc.line(input, points[0], points[3], zcol, thickness);

        // box
        Imgproc.line(input, points[4], points[5], xcol, thickness);
        Imgproc.line(input, points[6], points[7], xcol, thickness);
        Imgproc.line(input, points[8], points[9], xcol, thickness);
        Imgproc.line(input, points[10], points[11], xcol, thickness);
        Imgproc.line(input, points[4], points[6], ycol, thickness);
        Imgproc.line(input, points[5], points[7], ycol, thickness);
        Imgproc.line(input, points[8], points[10], ycol, thickness);
        Imgproc.line(input, points[9], points[11], ycol, thickness);
        Imgproc.line(input, points[4], points[8], zcol, thickness);
        Imgproc.line(input, points[5], points[9], zcol, thickness);
        Imgproc.line(input, points[6], points[10], zcol, thickness);
        Imgproc.line(input, points[7], points[11], zcol, thickness);

        Imgproc.putText(input, Integer.toString(detection.id), detection.center, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), thickness);

        double distanceFromCamera = Math.sqrt(pose.tvec.dot(pose.tvec));
        Imgproc.putText(input, Double.toString(distanceFromCamera), detection.corners[0], Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), thickness);

        pointsToProject.release();
        projectedPoints.release();
    }
}
