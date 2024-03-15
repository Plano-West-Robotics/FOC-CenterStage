package org.firstinspires.ftc.teamcode.poser.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.openftc.apriltag.AprilTagDetectorJNI;

@TeleOp(group = "test")
public class LocalizerTest extends OpMode {
    Hardware hardware;
    Localizer localizer;
    Drive drive;

    @Override
    public void init() {
        hardware = new Hardware(this);

        hardware.ledLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.ledRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

//        AprilTagDetector detector1 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        AprilTagDetector detector2 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        localizer = new KalmanFilter(
                Pose.ZERO,
                new ThreeDeadWheelLocalizer(hardware),
//                new AprilTagLocalizer(detector1, hardware, AprilTagLocalizer.Camera.FRONT),
                new AprilTagLocalizer(detector2, hardware, AprilTagLocalizer.Camera.REAR)
        );
        drive = new Drive(hardware, 0.35);

        FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);
    }

    @Override
    public void loop() {
        drive.driveOld(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        localizer.update();
        Pose p = localizer.getPoseEstimate();

        hardware.dashboardTelemetry.drawRobot(p);
        telemetry.addData("pose", p);
        telemetry.update();
    }
}
