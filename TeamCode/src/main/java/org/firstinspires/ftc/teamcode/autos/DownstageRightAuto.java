package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Downstage Right Auto")
public class DownstageRightAuto extends DownstageAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.BLUE);
    }
}
