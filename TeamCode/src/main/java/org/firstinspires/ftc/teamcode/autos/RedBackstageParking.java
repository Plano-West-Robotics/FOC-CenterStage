package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.InchWorm;
import org.firstinspires.ftc.teamcode.inchworm.InchWormBackup;
import org.firstinspires.ftc.teamcode.inchworm.WormUtil;
import org.firstinspires.ftc.teamcode.inchworm.units.Angle;
import org.firstinspires.ftc.teamcode.inchworm.units.Distance;

@Autonomous
public class RedBackstageParking extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWormBackup inchWorm = new InchWormBackup(this);

        wormUtil.waitForStart();

//        inchWorm.moveTo(new InchWorm.Pose(Distance.tiles(-4), Distance.ZERO, Angle.ZERO));
        inchWorm.moveTo(0, 5);
        inchWorm.moveTo(-48, 5);
    }
}
