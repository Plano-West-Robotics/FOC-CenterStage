package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Upstage Right Auto")
public class UpstageRightAuto extends UpstageAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.BLUE);
    }
}
