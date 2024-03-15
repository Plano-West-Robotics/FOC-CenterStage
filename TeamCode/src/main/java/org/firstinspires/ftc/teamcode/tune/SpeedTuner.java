package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.localization.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(group = "tune")
public class SpeedTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(this);
        Localizer localizer = new Localizer.FromDelta(new ThreeDeadWheelLocalizer(hardware), Pose.ZERO);
        Drive drive = new Drive(hardware, 1);

        waitForStart();

        double lastX = 0;
        double veloSum = 0;
        int numMeasurements = 0;

        ElapsedTime timer = new ElapsedTime();
        double time = getRuntime() + 1;
        drive.driveOld(0, 1, 0);
        while (getRuntime() < time) {
            double x = localizer.getPoseEstimate().pos.x.valInMM();

            double velo = (x - lastX) / timer.seconds();
            veloSum += velo;
            numMeasurements++;

            timer.reset();
            lastX = x;
            localizer.update();
        }

        drive.stop();
        time = getRuntime() + 1;
        drive.driveOld(0, 0, 1);

        double angSum = 0;
        int angMeasurements = 0;

        while (getRuntime() < time) {
            double current = Math.abs(hardware.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            angSum += current;
            angMeasurements++;
        }

        drive.stop();
        while (opModeIsActive()) {
            telemetry.addData("max velocity in mm / s", veloSum / numMeasurements);
            telemetry.addData("max angular velocity", angSum / angMeasurements);
            telemetry.update();
        }
    }
}
