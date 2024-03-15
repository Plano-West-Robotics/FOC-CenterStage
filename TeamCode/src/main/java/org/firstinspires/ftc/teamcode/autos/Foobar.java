package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.poser.Distance;

@Config
@Autonomous
public class Foobar extends AutoBase {
    public static double FOO = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        super.setup(Alliance.RED, UpOrDownStage.UPSTAGE);
        poser.setSpeed(poser.getSpeed()/2);

        waitForStart();

        while (true) {
            double foo = FOO;
            poser.moveBy(Distance.ZERO, Distance.inTiles(foo)).run();
            poser.moveBy(Distance.ZERO, Distance.inTiles(-foo)).run();
        }
    }
}
