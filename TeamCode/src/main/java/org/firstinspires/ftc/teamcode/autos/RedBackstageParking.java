package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.InchWorm;
import org.firstinspires.ftc.teamcode.inchworm.WormUtil;
import org.firstinspires.ftc.teamcode.inchworm.units.Angle;
import org.firstinspires.ftc.teamcode.inchworm.units.Distance;

@Autonomous
public class RedBackstageParking extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWorm inchWorm = new InchWorm(this, InchWorm.GLOBAL_ORIENTATION, InchWorm.POSE_ZERO);

        wormUtil.waitForStart();

        inchWorm.moveTo(new InchWorm.Pose(Distance.ZERO, Distance.tiles(4), Angle.ZERO));
    }
}
