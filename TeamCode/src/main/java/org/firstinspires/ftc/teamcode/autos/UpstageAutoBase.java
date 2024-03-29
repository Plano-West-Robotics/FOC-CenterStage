package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.Poser;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ControlledLift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public abstract class UpstageAutoBase extends AutoBase {
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
                        Distance.inTiles(0.5).add(Distance.inInches(4.5)),
                        Distance.inTiles(-1.5).add(Distance.inInches(2))
                ).run();
                break;
            case RIGHT:
                yOffsetAtBackdrop = Distance.inInches(-6);
                poser.goTo(
                        Distance.inTiles(1),
                        Distance.inTiles(-2).add(Distance.inInches(3))
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
            case MIDDLE:
            case RIGHT:
                poser.moveBy(
                        Distance.ZERO,
                        Distance.inInches(-3)
                ).run();
                break;
            default:
                break;
        }

        // go in front of backdrop
        poser.goTo(
                Distance.inTiles(1.5),
                Distance.inTiles(-2)
        ).run();
        poser.goTo(Angle.BACKWARD).run();
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
                Distance.inInches(4),
                Distance.ZERO
        ).run();
        // drop pixel
        arm.arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(1000);
        arm.arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        // move over
        poser.moveBy(
                Distance.inInches(-4),
                Distance.ZERO
        ).run();
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-2.5)
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
                Distance.inTiles(-2.5)
        ).run();

        // all done!
    }
}
