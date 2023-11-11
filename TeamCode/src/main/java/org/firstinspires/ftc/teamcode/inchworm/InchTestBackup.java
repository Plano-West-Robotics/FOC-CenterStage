package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class InchTestBackup extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil api = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION); // this is so cursed - karx 2023-11-10
        InchWormBackup inchWorm = new InchWormBackup(this);

        api.waitForStart();

        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(24, 24);
        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(0, 0, Math.PI / 2);

        inchWorm.moveTo(0, 0, 0);
    }
}