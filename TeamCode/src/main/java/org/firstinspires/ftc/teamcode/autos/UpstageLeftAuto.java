package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Upstage Left Auto")
public class UpstageLeftAuto extends UpstageAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        this.runOpMode(Alliance.RED);
    }
}
