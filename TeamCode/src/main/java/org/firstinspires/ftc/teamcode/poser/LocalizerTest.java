package org.firstinspires.ftc.teamcode.poser;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(group = "test")
public class LocalizerTest extends OpMode {
    Hardware hardware;
    Poser poser;
    Drive drive;

    @Override
    public void init() {
        hardware = new Hardware(this);
        poser = new Poser(hardware, 1, false, Pose.ZERO);
        FtcDashboard db = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, db.getTelemetry());
        drive = new Drive(hardware, 0.35);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        poser.localizer.update();

        Pose p = poser.localizer.getPoseEstimate();

        hardware.dashboardTelemetry.drawRobot(p);

        telemetry.addData("pose", p);
        telemetry.update();
    }
}
