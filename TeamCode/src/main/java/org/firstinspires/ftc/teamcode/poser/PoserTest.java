package org.firstinspires.ftc.teamcode.poser;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp
public class PoserTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(this);
        Poser poser = new Poser(hardware, 0.5, false, new Pose(Distance2.ZERO, Angle.ZERO));

        waitForStart();

        while (opModeIsActive()) {
            poser.goTo(new Pose(
                    Distance2.inTiles(0, 1),
                    Angle.inDegrees(-30)
            )).run();
            poser.goTo(new Pose(
                    Distance2.inTiles(0, 0),
                    Angle.ZERO
            )).run();
        }
    }
}
