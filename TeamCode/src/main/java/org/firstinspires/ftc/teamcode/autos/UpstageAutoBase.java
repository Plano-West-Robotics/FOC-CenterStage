package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.ConcurrentSet;
import org.firstinspires.ftc.teamcode.macro.ControlFlow;
import org.firstinspires.ftc.teamcode.macro.RunUntil;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.Poser;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.ControlledLift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public abstract class UpstageAutoBase extends AutoBase {
    public void runOpMode(Alliance alliance) {
        super.setup(alliance, UpOrDownStage.UPSTAGE);
        FreeSightPipeline.Side randomization = super.runVisionUntilStart();

//        poser.goTo(Distance2.inTiles(0.5, -2.5)).run();

        Distance yOffsetAtBackdrop = Distance.ZERO;
        switch (randomization) {
            case LEFT:
                yOffsetAtBackdrop = Distance.inInches(6);
//                RunUntil.firstCompletes(
//                        Wait.millis(500),
//                        poser.goTo(Distance.inTiles(1), Distance.inTiles(-1.5))
//                ).run();
//                poser.goTo(
//                        Distance.inTiles(0.5).sub(Distance.inInches(2.5)),
//                        Distance.inTiles(-1.5).add(Distance.inInches(2.5)),
//                        Angle.BACKWARD
//                ).run();
                poser.goTo(
                        Distance.inTiles(0.5),
                        Distance.inTiles(-1.5)
                ).run();
                poser.goTo(
                        Angle.BACKWARD
                ).run();
                poser.moveBy(
                        Distance.inInches(-2.5),
                        Distance.inInches(1)
                ).run();
                break;
            case MIDDLE:
//                poser.goTo(
//                        Distance.inTiles(1),
//                        Distance.inTiles(-1).sub(Distance.inInches(3)),
//                        Angle.BACKWARD.sub(Angle.inRadians(Math.PI / 16))
//                ).run();
                poser.goTo(
                        Distance.inTiles(1),
                        Distance.inTiles(-1).sub(Distance.inInches(3))
                ).run();
                poser.goTo(
                        Angle.BACKWARD.sub(Angle.inRadians(Math.PI / 16))
                ).run();
                break;
            case RIGHT:
                yOffsetAtBackdrop = Distance.inInches(-6);
//                poser.goTo(
//                        Distance.inTiles(1.5).sub(Distance.inInches(4)),
//                        Distance.inTiles(-1.5).add(Distance.inInches(1)),
//                        Angle.BACKWARD
//                ).run();
                poser.goTo(
                        Distance.inTiles(1.5).sub(Distance.inInches(4)),
                        Distance.inTiles(-1.5).add(Distance.inInches(1))
                ).run();
                poser.goTo(
                        Angle.BACKWARD
                ).run();
                break;
        }

        intake.reverse();
        intake.start();
        intake.update();
        sleep(250);
        intake.stop();
        intake.reverse();
        intake.update();

        ConcurrentSet.of(
                // go in front of backdrop
                Sequence.of(
                        poser.goTo(
                                Distance.inTiles(2).add(Distance.inInches(3)),
                                Distance.inTiles(-1.5).add(yOffsetAtBackdrop)
                        ),
                        poser.goTo(
                                Angle.BACKWARD
                        )
                ),
                // lift up
                Sequence.of(
                        Action.fromFn(() -> {
                            lift.setTarget(0.35);
                            arm.moveUp();
                        }),
                        new Action() {
                            @Override
                            public ControlFlow update() {
                                arm.update();
                                return ControlFlow.continueIf(lift.update() || arm.isBusy());
                            }

                            @Override
                            public void end() {
                                lift.stop();
                            }
                        }
                )
        ).run();

        // drop pixel
        box.setFlapPosition(Box.FlapPosition.OPEN);
        sleep(500);
        box.setFlapPosition(Box.FlapPosition.CLOSED);

        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-2.5)
        ).run();

        // lower slide and move arm in
        arm.moveDown();
        while (arm.isBusy()) arm.update();
        box.setFlapPosition(Box.FlapPosition.OPEN);
        sleep(500);
        lift.setTarget(0.00);
        while (lift.update());
        lift.stop();

        // move to final parking position
        poser.goTo(
                Distance.inTiles(2.5),
                Distance.inTiles(-2.5)
        ).run();

        poser.goTo(Angle.LEFT);

//        // all done!

//        // move over
//        poser.moveBy(
//                Distance.inInches(-3),
//                Distance.ZERO
//        ).run();
//        poser.goTo(
//                Distance.inTiles(2),
//                Distance.inTiles(-0.4)
//        ).run();
//
//        // lower slide and move arm in
//        arm.moveDown();
//        while (arm.isBusy()) arm.update();
//        box.setFlapPosition(Box.FlapPosition.OPEN);
//        sleep(500);
//        lift.setTarget(0.02);
//        while (lift.update());
//        lift.stop();
//
//        // move to final parking position
//        poser.goTo(
//                Distance.inTiles(2.5),
//                Distance.inTiles(-0.4)
//        ).run();

        poser.goTo(Angle.LEFT);

        // all done!
    }
}
