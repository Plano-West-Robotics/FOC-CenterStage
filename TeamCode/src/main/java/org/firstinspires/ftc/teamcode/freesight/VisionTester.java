package org.firstinspires.ftc.teamcode.freesight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.autos.Alliance;
import org.firstinspires.ftc.teamcode.autos.Vision;

@Autonomous(group = "test")
public class VisionTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(this);
        FtcDashboard db = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, db.getTelemetry());

        Vision vision = new Vision(hardware, Alliance.RED);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Detection", vision.getSide());
            telemetry.update();
        }

        vision.end();
    }
}
