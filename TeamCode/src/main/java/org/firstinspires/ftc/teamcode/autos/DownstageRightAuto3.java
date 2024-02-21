package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Downstage Right 2+0 Truss Auto")
public class DownstageRightAuto3 extends DownstageAuto3Base {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.BLUE);
    }
}
