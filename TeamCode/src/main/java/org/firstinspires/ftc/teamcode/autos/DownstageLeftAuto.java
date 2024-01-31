package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Downstage Left 1+0 Auto")
public class DownstageLeftAuto extends DownstageAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.RED);
    }
}
