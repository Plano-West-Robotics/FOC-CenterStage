package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ConstantProvider;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.opencv.core.Scalar;

@Autonomous
public class GrabberThingyOp extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     */
    @Override
    public void runOpMode()  {
        Hardware hardware = new Hardware(this);
        FreeSight vision = new FreeSight(hardware, this.telemetry);
        vision.init();
        FtcDashboard dash = FtcDashboard.getInstance();
//        dash.addConfigVariable(
//                "Scalar",
//                "high",
//                new ConstantProvider<>(vision.pipe.highHSV));
//        dash.addConfigVariable(
//                "Scalar",
//                "low",
//                new ConstantProvider<>(vision.pipe.lowHSV));
//        dash.addConfigVariable(
//                "Color",
//                "yeah",
//                new ConstantProvider<>(vision.pipe.colorState)
//                );


        waitForStart();
        while (opModeIsActive());

        vision.stop();
    }
}
