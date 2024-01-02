package org.firstinspires.ftc.teamcode.tune;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Localizer;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Config
@TeleOp(group = "tune")
public class TranslationalPIDTuner extends LinearOpMode {
    public static final double MAX_VEL = 939.571; // mm / s
    public static double TARGET = Distance.inTiles(1).valInMM();
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    /*
     * This class should be used to tune translational PID for InchWorm.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(this);
        Localizer localizer = new TwoDeadWheelLocalizer(hw, Pose.ZERO);
        Drive drive = new Drive(hw, 1);

        PIDController controller = new PIDController(Kp, Ki, Kd, TARGET);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            controller.setParams(Kp, Ki, Kd, TARGET);

            Pose current = localizer.getPoseEstimate();
            double out = controller.calculate(current.pos.y.valInMM());
            out /= MAX_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", TARGET - current.pos.y.valInMM()));
            telemetry.addData("current", String.format("%.2f", current.pos.y.valInMM()));
            telemetry.addData("target", String.format("%.2f", TARGET));
            telemetry.update();
            drive.drive(-out, 0, 0);
            localizer.update();
        }
    }
}
