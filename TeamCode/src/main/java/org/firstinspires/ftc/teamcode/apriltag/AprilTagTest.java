package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.teleop.Controls;
import org.opencv.core.MatOfDouble;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="AprilTag Test")
public class AprilTagTest extends OpModeWrapper {
    private AprilTagPipeline pipeline;
    private Drive drive;

    @Override
    public void setup() {
        // 10 deg right, 20 deg down

        pipeline = new AprilTagPipeline(new Pose3D(
                new MatOfDouble(-9 * 25.4, -3 * 25.4, 3 * 25.4),
                AprilTagPipeline.AUDIENCE_WALL_RVEC
        ), telemetry);
        hardware.rearCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                hardware.rearCam.setPipeline(pipeline);
                hardware.rearCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);
            }

            @Override
            public void onError(int errorCode) {}
        });
        drive = new Drive(hardware, 0.3);

        hardware.ledLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.ledRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    @Override
    public void run() {
        Pose2D pose = pipeline.getPoseEstimate();
        if (pose != null) {
            hardware.dashboardTelemetry.drawRobot(new Pose(
                    Distance2.inMM(pose.x, pose.y),
                    Angle.inRadians(pose.yaw)
            ));
        }

        drive.drive(
                gamepads.getAnalogValue(Controls.STRAFE),
                gamepads.getAnalogValue(Controls.STRAIGHT),
                gamepads.getAnalogValue(Controls.TURN)
        );
    }
}
