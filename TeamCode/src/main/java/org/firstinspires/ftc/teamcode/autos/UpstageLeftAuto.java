package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

@Autonomous(name = "Upstage Left Auto")
public class UpstageLeftAuto extends UpstageAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(this);
        Intake intake = new Intake(hardware, 0.3);
        Arm arm = new Arm(hardware);
        arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        ControlledLift lift = new ControlledLift(hardware);

        Pose startingPose = new Pose(Distance2.inTiles(0.5, -2.5), Angle.LEFT);
        startingPose = startingPose.then(new Pose(
                new Distance2(
                        // in the ROBOT's coordinate scheme (b/c .then)
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(16)).neg(),
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(18)).neg()
                ).div(2),
                Angle.ZERO
        ));
        Poser poser = new Poser(hardware, 0.9, false, startingPose);

        Vision vision = new Vision(hardware, Alliance.RED);

        while (!isStarted()) {
            telemetry.addData("Detection", vision.getSide());
            telemetry.update();
        }

        FreeSightPipeline.Side randomization = vision.end();

        poser.goTo(Distance2.inTiles(0.5, -2.5)).run();
        // arm up
        arm.setArmPosition(Arm.ArmPosition.UP);

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
                Distance.inTiles(2),
                Distance.inTiles(-1.5).add(yOffsetAtBackdrop),
                Angle.BACKWARD
        ).run();

        // move closer to backdrop
        poser.moveBy(
                Distance.inInches(5.35),
                Distance.inInches(-2.25)
        ).run();
        // lift up
        lift.setTarget(0.25);
        while (lift.update());
        lift.stop();
        // drop pixel
        arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(1000);
        arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        // move over
        poser.moveBy(
                // these should be negative of lines 130 and 131, i think
                Distance.inInches(-5.35),
                Distance.inInches(2.5)
        ).run();
        poser.goTo(
                Distance.inTiles(2),
                Distance.inTiles(-2.6)
        ).run();

        // lower slide and move arm in
        arm.setArmPosition(Arm.ArmPosition.DOWN);
        arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(500);
        lift.setTarget(0.02);
        while (lift.update());
        lift.stop();

        // move to final parking position
        poser.moveBy(
                Distance.inTiles(0.5),
                Distance.ZERO
        ).run();

        // all done!
    }
}
