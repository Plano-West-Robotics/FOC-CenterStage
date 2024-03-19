package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Downstage Left 2+1 Auto")
public class DownstageLeftAuto4 extends DownstageAuto4Base {
    @Override
    public void runOpMode() {
        this.runOpMode(Alliance.RED);
    }
}
