package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Downstage Left 2+0 Truss Auto")
public class DownstageLeftAuto3 extends DownstageAuto3Base {
    @Override
    public void runOpMode() {
        this.runOpMode(Alliance.RED);
    }
}
