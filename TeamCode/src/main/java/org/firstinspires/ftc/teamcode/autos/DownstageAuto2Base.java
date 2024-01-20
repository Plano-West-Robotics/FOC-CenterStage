package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public abstract class DownstageAuto2Base extends AutoBase {
    public void runOpMode(Alliance alliance) {
        super.setup(alliance, UpOrDownStage.DOWNSTAGE);
        FreeSightPipeline.Side randomization = super.runVisionUntilStart();

        poser.goTo(Distance2.inTiles(-1.5, -2.5)).run();

        Distance yOffsetAtBackdrop = Distance.ZERO;
        switch (randomization) {
            case LEFT:
                yOffsetAtBackdrop = Distance.inInches(7);
                poser.goTo(
                        Distance.inTiles(-2).add(Distance.inInches(1.5)),
                        Distance.inTiles(-1).add(Distance.inInches(6))
                ).run();
                poser.goTo(Angle.RIGHT).run();
                break;
            case MIDDLE:
                poser.goTo(
                        Distance.inTiles(-1.5).sub(Distance.inInches(2.5)),
                        Distance.inTiles(-0.5).sub(Distance.inInches(3.5))
                ).run();
                poser.goTo(Angle.RIGHT).run();
                break;
            case RIGHT:
                yOffsetAtBackdrop = Distance.inInches(-7);
                poser.goTo(
                        Distance.inTiles(-1.5),
                        Distance.inTiles(-1.5)
                ).run();
                poser.goTo(
                        Angle.FORWARD
                ).run();
                poser.moveBy(
                        Distance.inInches(2.5),
                        Distance.inInches(1)
                ).run();
                break;
        }

        intake.reverse();
        intake.start();
        intake.update();
        sleep(500);
        intake.stop();
        intake.reverse();
        intake.update();

        Distance crossingYCoord = Distance.inTiles(-0.5).add(Distance.inInches(1));
        switch (randomization) {
            case LEFT:
            case MIDDLE:
                break;
            case RIGHT:
                poser.moveBy(
                        Distance.inInches(-4),
                        Distance.ZERO
                ).run();
                break;
        }
        poser.goToY(crossingYCoord).run();
        poser.goTo(Angle.BACKWARD.sub(Angle.inDegrees(5))).run();
        // now in position to cross to the other side of the field

        // configurable delay goes here, if you wish

        // migrate
        poser.goTo(
                Distance.inTiles(2),
                crossingYCoord,
                Angle.BACKWARD
        ).run();
        // go in front of backdrop
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-1.5).add(yOffsetAtBackdrop)
        ).run();

        // lift up
        lift.setTarget(0.30);
        arm.moveUp();
        while (lift.update() && arm.isBusy()) arm.update();
        lift.stop();
        // move closer to backdrop
        poser.moveBy(
                Distance.inInches(4.75),
                Distance.ZERO
        ).run();
        // drop pixel
        arm.arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(1000);
        arm.arm.setFlapPosition(Arm.FlapPosition.CLOSED);

        // move over
        poser.moveBy(
                Distance.inInches(-4.75),
                Distance.ZERO
        ).run();
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-0.4)
        ).run();

        // lower slide and move arm in
        arm.moveDown();
        while (arm.isBusy()) arm.update();
        arm.arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(500);
        lift.setTarget(0.02);
        while (lift.update());
        lift.stop();

        // move to final parking position
        poser.goTo(
                Distance.inTiles(2.5),
                Distance.inTiles(-0.4)
        ).run();
    }
}
