package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ConstantProvider;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class GrabberThingyOp extends LinearOpMode {
    public static Scalar HIGH_HSV;
    public static Scalar LOW_HSV;
    public static FreeSightPipeline.Prop COLOR_STATE;

    @Override
    public void runOpMode()  {
        Hardware hardware = new Hardware(this);
        FreeSight vision = new FreeSight(hardware, this.telemetry);
        vision.init();
        FtcDashboard dash = FtcDashboard.getInstance();


        waitForStart();
        while (opModeIsActive()) {
            vision.pipe.highHSV = HIGH_HSV;
            vision.pipe.lowHSV = LOW_HSV;
            vision.pipe.colorState = COLOR_STATE;
        }

        vision.stop();
    }
}
