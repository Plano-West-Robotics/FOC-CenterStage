package org.firstinspires.ftc.teamcode.apriltag;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline {
    private final AprilTagDetector detector;
    private final Telemetry telemetry;

    private final Mat gray = new Mat();
    private final Pose3D tmp = new Pose3D();

    private final AprilTagDetector.LensIntrinsics intrinsics = AprilTagDetector.LensIntrinsics.FOR_C270_CAMERA_AT_800X448;
    private final Mat cameraMatrix;
    private final Pose3D robotPoseInCameraSpace;

//    private static class TagLocation {
//        public double size;
//        public Pose pose;
//
//        public TagLocation(double size, Pose pose) {
//            this.size = size;
//            this.pose = pose;
//        }
//    }

    private static final double MM_PER_INCH = 25.4;
//    private static final Pose DUMMY_POSE = new Pose(new MatOfDouble(), new MatOfDouble()); // TODO: actually put poses of every tag
//    private static final TagLocation[] TAGS = new TagLocation[] {
//            // Blue Alliance Backdrop (1 = left, 2 = center, 3 = right)
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//
//            // Red Alliance Backdrop (4 = left, 5 = center, 6 = right)
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//            new TagLocation(2 * MM_PER_INCH, DUMMY_POSE),
//
//            // Red Alliance Audience Wall (Tile E1)
//            new TagLocation(5 * MM_PER_INCH, new Pose(
//                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, -41 * MM_PER_INCH, 5.5 * MM_PER_INCH),
//                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
//            )),
//            new TagLocation(2 * MM_PER_INCH, new Pose(
//                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, -35.5 * MM_PER_INCH, 4 * MM_PER_INCH),
//                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
//            )),
//
//            // Blue Alliance Audience Wall (Tile B1)
//            new TagLocation(2 * MM_PER_INCH, new Pose(
//                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 35.5 * MM_PER_INCH, 4 * MM_PER_INCH),
//                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
//            )),
//            new TagLocation(5 * MM_PER_INCH, new Pose(
//                    new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 41 * MM_PER_INCH, 5.5 * MM_PER_INCH),
//                    new MatOfDouble(-TWO_PI_OVER_THREE_ROOT_THREE, -TWO_PI_OVER_THREE_ROOT_THREE, TWO_PI_OVER_THREE_ROOT_THREE)
//            )),
//    };


    public static final MatOfDouble AUDIENCE_WALL_RVEC;
    static {
        double a = 2 * Math.PI / (3 * Math.sqrt(3));
        AUDIENCE_WALL_RVEC = new MatOfDouble(-a, -a, a);
    }
    public static final MatOfDouble BACKDROP_RVEC;
    static {
        double a = Math.sqrt(3);
        double b = 2 * Math.asin(Math.sqrt(7/8.)) / Math.sqrt(7);
        BACKDROP_RVEC = new MatOfDouble(-a * b, a * b, -b);
    }
    public static final TagGroup[] TAG_GROUPS = new TagGroup[] {
            // Blue Alliance Backdrop
            new TagGroup(
                    new Pose3D(
                            new MatOfDouble((70.5 - 11) * MM_PER_INCH, 35.4375 * MM_PER_INCH, 0),
                            BACKDROP_RVEC
                    ),
                    new TagGroup.Tag(1, new Point(-6 * MM_PER_INCH, -5 * MM_PER_INCH), 2 * MM_PER_INCH),
                    new TagGroup.Tag(2, new Point(0, -5 * MM_PER_INCH), 2 * MM_PER_INCH),
                    new TagGroup.Tag(3, new Point(6 * MM_PER_INCH, -5 * MM_PER_INCH), 2 * MM_PER_INCH)
            ),
            // Red Alliance Backdrop
            new TagGroup(
                    new Pose3D(
                            new MatOfDouble((70.5 - 11) * MM_PER_INCH, -35.4375 * MM_PER_INCH, 0),
                            BACKDROP_RVEC
                    ),
                    new TagGroup.Tag(4, new Point(-6 * MM_PER_INCH, -5 * MM_PER_INCH), 2 * MM_PER_INCH),
                    new TagGroup.Tag(5, new Point(0, -5 * MM_PER_INCH), 2 * MM_PER_INCH),
                    new TagGroup.Tag(6, new Point(6 * MM_PER_INCH, -5 * MM_PER_INCH), 2 * MM_PER_INCH)
//            ),
            // Red Alliance Audience Wall (Tile E1)
//            new TagGroup(
//                    new Pose3D(
//                            new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, -35.4375 * MM_PER_INCH, 4 * MM_PER_INCH),
//                            AUDIENCE_WALL_RVEC
//                    ),
//                    new TagGroup.Tag(7, new Point(-5.5 * MM_PER_INCH, -1.5 * MM_PER_INCH), 5 * MM_PER_INCH),
//                    new TagGroup.Tag(8, new Point(0, 0), 2 * MM_PER_INCH)
//            ),
            // Blue Alliance Audience Wall (Tile B1)
//            new TagGroup(
//                    new Pose3D(
//                            new MatOfDouble(-(70.5 + 1) * MM_PER_INCH, 35.4375 * MM_PER_INCH, 4 * MM_PER_INCH),
//                            AUDIENCE_WALL_RVEC
//                    ),
//                    new TagGroup.Tag(9, new Point(0, 0), 2 * MM_PER_INCH),
//                    new TagGroup.Tag(10, new Point(5.5 * MM_PER_INCH, -1.5 * MM_PER_INCH), 5 * MM_PER_INCH)
            )
    };

    private static final int MAX_IDX = 9;
    private static int idxFromId(int id) {
        if (id >= 1 && id <= 10) return id - 1;
        else return -1;
    }

    private Pose2D poseEstimate = null;
    private final Object lock = new Object();

    // only for use in the simulator
    public AprilTagPipeline(Telemetry telemetry) {
        this(
                new Pose3D(
                        new MatOfDouble(0, 0, 0),
                        new MatOfDouble(-2*Math.PI/(3*Math.sqrt(3)), 2*Math.PI/(3*Math.sqrt(3)), -2*Math.PI/(3*Math.sqrt(3)))
                ),
                telemetry
        );
    }

    public AprilTagPipeline(Pose3D cameraPose, Telemetry telemetry) {
        this(
                new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3),
                cameraPose,
                telemetry
        );
    }

    public AprilTagPipeline(AprilTagDetector detector, Pose3D cameraPose, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.detector = detector;

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

        cameraPose = cameraPose.duplicate();
        cameraPose.invert();
        this.robotPoseInCameraSpace = cameraPose;
    }

    /**
     * @param input the frame to be manipulated
     * @return The altered matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
        AprilTagDetector.AprilTagDetections detections = this.detector.run(gray, this.intrinsics);

        AprilTagDetector.AprilTagDetection[] detectionsArr = new AprilTagDetector.AprilTagDetection[MAX_IDX + 1];
        for (AprilTagDetector.AprilTagDetection detection : detections) {
            int idx = idxFromId(detection.id);
            if (idx >= 0 && idx <= MAX_IDX) {
                telemetry.addLine("we see april tag " + detection.id);
                detectionsArr[idx] = detection;
            }
        }

        // for averaging
        double xTotal = 0;
        double yTotal = 0;
        double yawTotal = 0;
        double count = 0;

        for (int i = 0; i < TAG_GROUPS.length; i++) {
            TagGroup group = TAG_GROUPS[i];

            ArrayList<Point3> objectPoints = new ArrayList<>();
            ArrayList<Point> imagePoints = new ArrayList<>();

            boolean anyTagsFound = false;
            for (TagGroup.Tag tag : group.tags) {
                AprilTagDetector.AprilTagDetection detection = detectionsArr[idxFromId(tag.id)];
                if (detection == null) continue;

                anyTagsFound = true;

                objectPoints.add(new Point3(
                        tag.pos.x,
                        tag.pos.y,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x - tag.size / 2,
                        tag.pos.y + tag.size / 2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x + tag.size / 2,
                        tag.pos.y + tag.size / 2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x + tag.size / 2,
                        tag.pos.y - tag.size / 2,
                        0
                ));
                objectPoints.add(new Point3(
                        tag.pos.x - tag.size / 2,
                        tag.pos.y - tag.size / 2,
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

            telemetry.addLine("we see group " + i);

            MatOfPoint3f objectPointsMat = new MatOfPoint3f();
            objectPointsMat.fromList(objectPoints);
            MatOfPoint2f imagePointsMat = new MatOfPoint2f();
            imagePointsMat.fromList(imagePoints);

            Pose3D pose = tmp;
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

            drawStuffIdk(input, objectPoints, pose);

            pose.invert();
            pose.composeAfter(group.pose);
            pose.compose(robotPoseInCameraSpace);
//            telemetry.addLine("group " + i + " tvec " + pose.tvec.dump());
//            telemetry.addLine("group " + i + " rvec " + pose.rvec.dump());
            Double yaw = yawFromRobotPose(pose);
            if (yaw == null) {
//                telemetry.addLine("group " + i + " rvec was too crazy");
                // the robot is pitched or rolled too much to be reasonable
                continue;
            }

            double x = pose.tvec.get(0, 0)[0];
            double y = pose.tvec.get(1, 0)[0];
            double z = pose.tvec.get(2, 0)[0];

            if (z <= -75 || z >= 75) {
//                telemetry.addLine("group " + i + " z was too crazy");
                // the robot is in the ground or in the air too far to be reasonable
                continue;
            }

//            telemetry.addLine("group " + i + " x " + x);
//            telemetry.addLine("group " + i + " y " + y);
//            telemetry.addLine("group " + i + " yaw " + yaw);

            xTotal += x;
            yTotal += y;
            yawTotal += yaw;
            count += 1;
        }

        if (count > 0) {
            synchronized (lock) {
                poseEstimate = new Pose2D(
                        xTotal / count,
                        yTotal / count,
                        yawTotal / count
                );
            }
        } else {
            synchronized (lock) {
                poseEstimate = null;
            }
        }

        return input;
    }

    public Pose2D getPoseEstimate() {
        synchronized (lock) {
            if (poseEstimate == null) return null;
            return new Pose2D(poseEstimate.x, poseEstimate.y, poseEstimate.yaw);
        }
    }

    private Double yawFromRobotPose(Pose3D pose) {
        double ourX = pose.rvec.get(0, 0)[0];
        double ourY = pose.rvec.get(1, 0)[0];
        double ourZ = pose.rvec.get(2, 0)[0];
        double x = -ourY; // HACK: convert from our axes system to the one used by the website i copied from
        double y = ourZ;
        double z = -ourX;
        double angle = Math.sqrt(x * x + y * y + z * z);
        if (angle == 0) return 0.0;
        x /= angle;
        y /= angle;
        z /= angle;

        // sourced from http://euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm (with modifications)

        double s = Math.sin(angle);
        double c = Math.cos(angle);
        double t = 1 - c;

        // check pitch
        double h = x*y*t + z*s; // pitch = Math.asin(h);
        if (h > 0.2 || h < -0.2) { // ~11 degrees of pitch
            return null;
        }

        // check roll
        double v1 = 1 - (x*x + z*z) * t;
        if (v1 < 0) return null; // upside down
        double v2 = (x*s - y*z*t) / v1; // roll = Math.atan2(x*s - y*z*t, v1)
        if (v2 > 0.2 || v2 < -0.2) { // ~11 degrees of roll
            return null;
        }

        double yaw = Math.atan2(y*s - x*z*t, 1 - (y*y + z*z) * t);
        return yaw;
    }

    private void drawStuffIdk(Mat input, ArrayList<Point3> objectPoints, Pose3D pose) {
        ArrayList<Point3> objectPoints2 = new ArrayList<>();

        objectPoints2.add(new Point3(0, 0, 0));
        objectPoints2.add(new Point3(20, 0, 0));
        objectPoints2.add(new Point3(0, 20, 0));
        objectPoints2.add(new Point3(0, 0, -20));
        assert objectPoints.size() % 5 == 0;
        for (int i = 0; i < objectPoints.size(); i += 5) {
            Point3 center = objectPoints.get(i);
            Point3 corner1 = objectPoints.get(i + 1);
            Point3 corner0 = objectPoints.get(i + 2);

            double tagSize = corner0.x - corner1.x;
            double boxSize = tagSize * 5/4;

            objectPoints2.add(new Point3(boxSize / 2 + center.x, boxSize / 2 + center.y, 0));
            objectPoints2.add(new Point3(-boxSize / 2 + center.x, boxSize / 2 + center.y, 0));
            objectPoints2.add(new Point3(boxSize / 2 + center.x, -boxSize / 2 + center.y, 0));
            objectPoints2.add(new Point3(-boxSize / 2 + center.x, -boxSize / 2 + center.y, 0));
            objectPoints2.add(new Point3(boxSize / 2 + center.x, boxSize / 2 + center.y, -boxSize));
            objectPoints2.add(new Point3(-boxSize / 2 + center.x, boxSize / 2 + center.y, -boxSize));
            objectPoints2.add(new Point3(boxSize / 2 + center.x, -boxSize / 2 + center.y, -boxSize));
            objectPoints2.add(new Point3(-boxSize / 2 + center.x, -boxSize / 2 + center.y, -boxSize));
        }
        MatOfPoint3f pointsToProject = new MatOfPoint3f();
        pointsToProject.fromList(objectPoints2);

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

        for (int i = 4; i < points.length; i += 8) {
            Imgproc.line(input, points[i], points[i + 1], xcol, thickness);
            Imgproc.line(input, points[i + 2], points[i + 3], xcol, thickness);
            Imgproc.line(input, points[i + 4], points[i + 5], xcol, thickness);
            Imgproc.line(input, points[i + 6], points[i + 7], xcol, thickness);
            Imgproc.line(input, points[i], points[i + 2], ycol, thickness);
            Imgproc.line(input, points[i + 1], points[i + 3], ycol, thickness);
            Imgproc.line(input, points[i + 4], points[i + 6], ycol, thickness);
            Imgproc.line(input, points[i + 5], points[i + 7], ycol, thickness);
            Imgproc.line(input, points[i], points[i + 4], zcol, thickness);
            Imgproc.line(input, points[i + 1], points[i + 5], zcol, thickness);
            Imgproc.line(input, points[i + 2], points[i + 6], zcol, thickness);
            Imgproc.line(input, points[i + 3], points[i + 7], zcol, thickness);
        }

        pointsToProject.release();
        projectedPoints.release();
    }

    private void drawTag(Mat input, double tagSize, AprilTagDetector.AprilTagDetection detection, Pose3D pose) {
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
