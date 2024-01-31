package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Downstage Right 2+0 Auto")
public class DownstageRightAuto2 extends DownstageAuto2Base {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.BLUE);
    }
}
