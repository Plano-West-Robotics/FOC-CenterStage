package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.ConcurrentSet;
import org.firstinspires.ftc.teamcode.macro.Repeat;
import org.firstinspires.ftc.teamcode.macro.RunUntil;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.subsystems.Box;

public abstract class DownstageAuto4Base extends AutoBase {
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
        poser.goToY(Distance.inTiles(-0.5)).run();
        poser.goTo(Angle.BACKWARD).run();
        poser.goTo(
                Distance.inTiles(-2.5),
                Distance.inTiles(-0.5)
        ).run();

        RunUntil.firstCompletes(
                Sequence.of(
                        Wait.millis(750),
                        Action.fromFn(() -> {
                            box.toggleFlapPosition();
                            intake.start();
                            intake.update();
                        }),
                        Wait.millis(500),
                        Action.fromFn(() -> {
                            intake.reverse();
                            intake.update();
                        })
                ),
                Sequence.of(
                        Action.fromFn(() -> poser.move(0.2, 0, 0)),
                        Repeat.forever(Action.fromFn(() -> poser.localizer.update()))
                )
        ).run();
        ConcurrentSet.of(
                Sequence.of(
                        Wait.millis(500),
                        Action.fromFn(() -> {
                            intake.reverse();
                            intake.update();
                        }),
                        Wait.millis(750),
                        Action.fromFn(() -> {
                            box.toggleFlapPosition();
                            intake.stop();
                            intake.update();
                        })
                ),
                poser.goTo(
                        Distance.inTiles(-1.5),
                        Distance.inTiles(-0.5)
                )
        ).run();

        Distance crossingYCoord = Distance.inTiles(-0.5).add(Distance.inInches(1));
        poser.goToY(crossingYCoord).run();
        poser.goTo(Angle.BACKWARD.add(Angle.inDegrees(2.5))).run();
        // now in position to cross to the other side of the field

        // configurable delay goes here, if you wish
//        sleep(6000);

        // migrate
        poser.goTo(
                Distance.inTiles(1.6),
                crossingYCoord,
                Angle.BACKWARD
        ).run();
        // go in front of backdrop
        poser.goTo(
                Distance.inTiles(1.6),
                Distance.inTiles(-1.5).add(yOffsetAtBackdrop)
        ).run();
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-1.5).add(yOffsetAtBackdrop)
        ).run();

        // lift up
        lift.setTarget(0.35);
        arm.moveUp();
        while (lift.update() && arm.isBusy()) arm.update();
        lift.stop();
        // move closer to backdrop
        poser.moveBy(
                Distance.inInches(3),
                Distance.ZERO
        ).run();
        // drop pixel
        box.setFlapPosition(Box.FlapPosition.OPEN);
        sleep(1000);
        box.setFlapPosition(Box.FlapPosition.CLOSED);

        // move over
        poser.moveBy(
                Distance.inInches(-3),
                Distance.ZERO
        ).run();
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-0.4)
        ).run();

        // lower slide and move arm in
        arm.moveDown();
        while (arm.isBusy()) arm.update();
        box.setFlapPosition(Box.FlapPosition.OPEN);
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