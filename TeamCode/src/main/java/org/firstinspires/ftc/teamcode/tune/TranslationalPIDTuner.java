package org.firstinspires.ftc.teamcode.tune;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.PIDController;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.Poser;

@Config
@TeleOp(group = "tune")
public class TranslationalPIDTuner extends LinearOpMode {
    public static double TARGET = Distance.inTiles(1).valInMM();
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    /*
     * This class should be used to tune translational PID for Poser.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(this);
        Poser poser = new Poser(hw, 0.9, false, Pose.ZERO);

        waitForStart();

        Poser.TuningMotion motion = poser.new TuningMotion(Pose.ZERO);
        while (opModeIsActive()) {
            motion.setTransCoeffs(Kp, Ki, Kd);
            motion.setTarget(new Pose(
                    new Distance2(Distance.inMM(TARGET), Distance.ZERO),
                    Angle.ZERO
            ));

            motion.update();

            telemetry.addData("current", String.format("%.2f", poser.localizer.getPoseEstimate().pos.x.valInMM()));
            telemetry.addData("target", String.format("%.2f", TARGET));
            telemetry.update();
        }
    }
}
