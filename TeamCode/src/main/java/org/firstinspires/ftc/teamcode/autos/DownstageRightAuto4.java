package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Downstage Right 2+1 Auto")
public class DownstageRightAuto4 extends DownstageAuto4Base {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.BLUE);
    }
}
