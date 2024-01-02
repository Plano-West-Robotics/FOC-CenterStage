package org.firstinspires.ftc.teamcode.poser;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(group = "test")
public class LocalizerTest extends OpMode {
    Hardware hardware;
    Localizer localizer;
    Drive drive;

    @Override
    public void init() {
        hardware = new Hardware(this);
        localizer = new TwoDeadWheelLocalizer(hardware, Pose.ZERO);
        drive = new Drive(hardware, 0.35);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        localizer.update();
        Pose p = localizer.getPoseEstimate();

        hardware.dashboardTelemetry.drawRobot(p);
        telemetry.addData("pose", p);
        telemetry.update();
    }
}
