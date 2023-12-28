package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Sensor;

public class ColorSensorTester extends LinearOpMode {
    Hardware hardware;
    @Override
    public void runOpMode() {
        hardware = new Hardware(this);

        waitForStart();
        while (opModeIsActive()) {
            Sensor.ColorScalar top_color = new Sensor.ColorScalar(
                    hardware.top.red(),
                    hardware.top.green(),
                    hardware.top.blue()
            );
            top_color.cvtRGBToHSV();
            telemetry.addLine("TOP SENSOR");
            telemetry.addData("h", top_color.val[0]);
            telemetry.addData("s", top_color.val[1]);
            telemetry.addData("v", top_color.val[2]);

            Sensor.ColorScalar bottom_color = new Sensor.ColorScalar(
                    hardware.bottom.red(),
                    hardware.bottom.green(),
                    hardware.bottom.blue()
            );
            bottom_color.cvtRGBToHSV();
            telemetry.addLine("BOTTOM SENSOR");
            telemetry.addData("h", bottom_color.val[0]);
            telemetry.addData("s", bottom_color.val[1]);
            telemetry.addData("v", bottom_color.val[2]);

           telemetry.update();
        }
    }
}
