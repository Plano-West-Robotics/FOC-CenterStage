package org.firstinspires.ftc.teamcode.tune;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.localization.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Config
@TeleOp(group = "tune")
public class TurnPIDTuner extends LinearOpMode {
    public static final double MAX_ANG_VEL = -156.941;
    public static double TARGET = 90;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    /*
     * This class should be used to tune turn PID for InchWorm.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(this);
        Localizer localizer = new Localizer.FromDelta(new TwoDeadWheelLocalizer(hardware), Pose.ZERO);
        Drive drive = new Drive(hardware, 1);

        PIDController controller = new PIDController(Kp, Ki, Kd, TARGET);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            controller.setParams(Kp, Ki, Kd, TARGET);

            Angle current = localizer.getPoseEstimate().yaw;
            double out = controller.calculateWithError(Angle.inDegrees(TARGET).sub(current).valInDegrees());
            out /= MAX_ANG_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", Angle.inDegrees(TARGET).sub(current).valInDegrees()));
            telemetry.addData("current", String.format("%.2f", current.valInDegrees()));
            telemetry.addData("target", String.format("%.2f", TARGET));
            telemetry.update();

            drive.drive(0, 0, out);
            localizer.update();
        }
    }
}
