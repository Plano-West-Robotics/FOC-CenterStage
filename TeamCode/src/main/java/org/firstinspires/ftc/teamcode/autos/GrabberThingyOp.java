package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;

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
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode()  {
        FreeSight vision = new FreeSight(this);
        vision.init();

        waitForStart();
        while (opModeIsActive());

        vision.stop();
    }
}
