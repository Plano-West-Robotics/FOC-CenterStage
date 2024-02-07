package org.firstinspires.ftc.teamcode.poser.localization;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.apriltag.Pose2D;
import org.firstinspires.ftc.teamcode.apriltag.Pose3D;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class AprilTagLocalizer implements FallibleLocalizer {
    public enum Camera {
        FRONT,
        REAR,
    }

    private final Hardware hardware;
    private final AprilTagPipeline pipeline;

    public AprilTagLocalizer(AprilTagDetector detector, Hardware hardware, Camera camera) {
        this.hardware = hardware;

        Pose3D cameraPose = null;
        OpenCvCamera openCvCamera2 = null;
        switch (camera) {
            case FRONT:
                cameraPose = Pose3D.FRONT_CAMERA_POSE;
                openCvCamera2 = hardware.frontCam;
                break;
            case REAR:
                cameraPose = Pose3D.REAR_CAMERA_POSE;
                openCvCamera2 = hardware.rearCam;
                break;
        }
        OpenCvCamera openCvCamera = openCvCamera2;

        pipeline = new AprilTagPipeline(
                detector,
                cameraPose,
                hardware.opMode.telemetry
        );

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCamera.setPipeline(pipeline);
                openCvCamera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public void update() { }

    public Pose getPoseEstimate() {
        if (
                Math.abs(hardware.fl.getPower()) > 0.1
                || Math.abs(hardware.fr.getPower()) > 0.1
                || Math.abs(hardware.bl.getPower()) > 0.1
                || Math.abs(hardware.br.getPower()) > 0.1
        ) {
            return null;
        }

        Pose2D pose2D = this.pipeline.getPoseEstimate();
        if (pose2D == null) return null;
        return new Pose(
                Distance2.inMM(pose2D.x, pose2D.y),
                Angle.inRadians(pose2D.yaw)
        );
    }
}
