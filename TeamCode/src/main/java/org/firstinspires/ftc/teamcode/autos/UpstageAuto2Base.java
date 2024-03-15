package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;

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
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Box;

@Config
public abstract class UpstageAuto2Base extends AutoBase {
    public static boolean FOO = true;
    
    public void runOpMode(Alliance alliance) {
        super.setup(alliance, UpOrDownStage.UPSTAGE);
        FreeSightPipeline.Side randomization = super.runVisionUntilStart();

        poser.goTo(Distance2.inTiles(0.5, -2.5)).run();

        Distance yOffsetAtBackdrop = Distance.ZERO;
        switch (randomization) {
            case LEFT:
                yOffsetAtBackdrop = Distance.inInches(6);
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
                poser.goTo(
                        Distance.inTiles(1),
                        Distance.inTiles(-1).sub(Distance.inInches(3)),
                        Angle.BACKWARD.sub(Angle.inRadians(Math.PI / 16))
                ).run();
                break;
            case RIGHT:
                yOffsetAtBackdrop = Distance.inInches(-6);
                poser.goTo(
                        Distance.inTiles(1.5).sub(Distance.inInches(4)),
                        Distance.inTiles(-1.5).add(Distance.inInches(1)),
                        Angle.BACKWARD
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

//        switch (randomization) {
//            case MIDDLE:
//            case RIGHT:
//                poser.moveBy(
//                        Distance.ZERO,
//                        Distance.inInches(-3)
//                ).run();
//                break;
//            default:
//                break;
//        }

        // go in front of backdrop
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-1.5).add(yOffsetAtBackdrop),
                Angle.BACKWARD
        ).run();

        // for cycling
        if (yOffsetAtBackdrop.isZero()) yOffsetAtBackdrop = Distance.inInches(6);

        Runnable dropPixels = () -> {
            // lift up
            lift.setTarget(0.30);
            arm.moveUp();
            while (lift.update() && arm.isBusy()) arm.update();
            lift.stop();
            // move closer to backdrop
            poser.moveBy(
                    Distance.inInches(4),
                    Distance.ZERO
            ).run();
            // drop pixel
            box.setFlapPosition(Box.FlapPosition.OPEN);
            sleep(1000);
            box.setFlapPosition(Box.FlapPosition.CLOSED);
            // move away from backdrop
            poser.moveBy(
                    Distance.inInches(-4),
                    Distance.ZERO
            ).run();
            // lower slide and move arm in
            arm.moveDown();
            while (arm.isBusy()) arm.update();
            box.setFlapPosition(Box.FlapPosition.OPEN);
            sleep(500);
            lift.setTarget(0.02);
            while (lift.update()) ;
            lift.stop();
        };
        final Distance finalYOffsetAtBackdrop = yOffsetAtBackdrop; // for the below closure
        Runnable takePixels = () -> {
            // start in front of the backdrop
            // move into the C/D file
            poser.setSpeed(poser.getSpeed()*2/3);
            RunUntil.firstCompletes(
                    Wait.seconds(9),
                    new Action() {
                        Action foo = poser.goTo(
                                Distance.inTiles(2),
                                Distance.inTiles(-0.4)
                        );

                        @Override
                        public ControlFlow update() {
                            foo.update();
                            return ControlFlow.CONTINUE;
                        }

                        @Override
                        public void end() {
                            foo.end();
                        }
                    }
            ).run();
            hardware.log.addLine("UpstageAuto2Base", "" + poser.localizer.getPoseEstimate());
            while (FOO);
            poser.setSpeed(poser.getSpeed()*3/2);
            // cross the field to the stack
            poser.setSpeed(poser.getSpeed()/2);
            poser.goTo(
                    Distance.inTiles(-1.5),
                    Distance.inTiles(-0.4)
            ).run();
            poser.setSpeed(poser.getSpeed()*2);
            poser.setSpeed(poser.getSpeed() / 3);
            poser.goTo(
                    Distance.inTiles(-2.5).sub(Distance.inInches(0.5)),
                    Distance.inTiles(-0.5)
            ).run();
            poser.setSpeed(poser.getSpeed() * 3);
            RunUntil.firstCompletes(
                    // pick up the pixels
                    Sequence.of(
                            Action.fromFn(() -> {
                                intake.start();
                                intake.update();
                            }),
                            Wait.millis(500),
                            Action.fromFn(() -> {
                                intake.reverse();
                                intake.update();
                            }),
                            Wait.millis(100),
                            Action.fromFn(() -> {
                                intake.reverse();
                                intake.stop();
                                intake.update();
                            })
                    ),
                    // while pushing against the stack
                    poser.moveBy(Distance.inInches(-3), Distance.ZERO)
            ).run();
            ConcurrentSet.of(
                    Sequence.of(
                            // move away from the stack
                            poser.goTo(
                                    Distance.inTiles(-1.5),
                                    Distance.inTiles(-0.4)
                            ),
                            // move back to the other side of the field
                            poser.goTo(
                                    Distance.inTiles(1.6),
                                    Distance.inTiles(-0.4)
                            ),
                            // and back in front of the backdrop
                            poser.goTo(
                                    Distance.inTiles(1.6),
                                    Distance.inTiles(-1.5).add(finalYOffsetAtBackdrop)
                            ),
                            poser.goToX(Distance.inTiles(2))
                    ),
                    // but also pull the pixels into the box at the same time
                    Sequence.of(
                            Wait.millis(300),
                            Action.fromFn(() -> {
                                box.setFlapPosition(Box.FlapPosition.OPEN);
                                intake.start();
                                intake.update();
                            }),
                            Wait.millis(2000),
                            Action.fromFn(() -> {
                                box.setFlapPosition(Box.FlapPosition.CLOSED);
                                intake.stop();
                                intake.update();
                            })
                    )
            ).run();
        };

        dropPixels.run(); // the yellow pixel
        for (int i = 0; i < 2; i++) {
            takePixels.run();
            dropPixels.run(); // this time its the white pixels
        }

        // into the corner
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-2.5)
        ).run();
        poser.goTo(
                Distance.inTiles(2.5),
                Distance.inTiles(-2.5)
        ).run();

        // all done!
    }
}
