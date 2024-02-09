package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Sensor;

@Autonomous(group = "test")
public class ColorSensorTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorRangeSensor top = hardwareMap.get(ColorRangeSensor.class, "top");
        ColorRangeSensor bottom = hardwareMap.get(ColorRangeSensor.class, "bottom");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        top.enableLed(true);
        bottom.enableLed(true);

        waitForStart();
        while (opModeIsActive()) {
            Sensor.ColorScalar top_color = new Sensor.ColorScalar(
                    top.getNormalizedColors().red,
                    top.getNormalizedColors().green,
                    top.getNormalizedColors().blue
            );
            top_color.cvtRGBToHSV();
            telemetry.addLine("TOP SENSOR");
            telemetry.addData("h", top_color.val[0]);
            telemetry.addData("s", top_color.val[1]);
            telemetry.addData("v", top_color.val[2]);
            telemetry.addData("Dist", top.getDistance(DistanceUnit.MM));
            telemetry.addData("Color", Sensor.getMost(top_color, top.getDistance(DistanceUnit.MM)).name());
            telemetry.addLine();

            Sensor.ColorScalar bottom_color = new Sensor.ColorScalar(
                    bottom.getNormalizedColors().red,
                    bottom.getNormalizedColors().green,
                    bottom.getNormalizedColors().blue
            );
            bottom_color.cvtRGBToHSV();
            telemetry.addLine("BOTTOM SENSOR");
            telemetry.addData("h", bottom_color.val[0]);
            telemetry.addData("s", bottom_color.val[1]);
            telemetry.addData("v", bottom_color.val[2]);
            telemetry.addData("Dist", bottom.getDistance(DistanceUnit.MM));
            telemetry.addData("Color", Sensor.getMost(bottom_color, bottom.getDistance(DistanceUnit.MM)).name());

           telemetry.update();
        }
    }
}
