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

public abstract class DownstageAutoBase extends LinearOpMode {
    public void runOpMode(Alliance alliance) {
        Hardware hardware = new Hardware(this);
        Intake intake = new Intake(hardware, 0.3);
        Arm arm = new Arm(hardware, telemetry);
        arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        ControlledLift lift = new ControlledLift(hardware);

        Pose startingPose = new Pose(Distance2.inTiles(-1.5, -2.5), Angle.LEFT);
        boolean isFlipped = alliance.isBlue();
        if (isFlipped) {
            startingPose = startingPose.flippedAcrossXAxis();
        }
        startingPose = startingPose.then(new Pose(
                new Distance2(
                        // in the ROBOT's coordinate scheme (b/c .then)
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(16)).neg(),
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(18)).neg()
                ).div(2),
                Angle.ZERO
        ));
        if (alliance.isRed()) {
            startingPose = startingPose.then(new Pose(
                    new Distance2(
                            Distance.ZERO,
                            Distance.inInches(2)
                    ),
                    Angle.ZERO
            ));
        }
        Poser poser = new Poser(hardware, 0.9, isFlipped, startingPose);

        Vision vision = new Vision(hardware, alliance);

        while (!isStarted()) {
            telemetry.addData("Detection", vision.getSide());
            telemetry.update();
        }

        FreeSightPipeline.Side randomization = vision.end();
        if (isFlipped) switch (randomization) {
            case LEFT:
                randomization = FreeSightPipeline.Side.RIGHT;
                break;
            case RIGHT:
                randomization = FreeSightPipeline.Side.LEFT;
                break;
            default:
                break;
        }

        poser.goTo(Distance2.inTiles(-1.5, -2.5)).run();

        switch (randomization) {
            case LEFT:
                poser.goTo(
                        Distance.inTiles(-2),
                        Distance.inTiles(-2).add(Distance.inInches(3))
                ).run();
                break;
            case MIDDLE:
                poser.goTo(
                        Distance.inTiles(-1.5).sub(Distance.inInches(4.5)),
                        Distance.inTiles(-1.5).add(Distance.inInches(4))
                ).run();
                break;
            case RIGHT:
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
                poser.moveBy(
                        Distance.ZERO,
                        Distance.inInches(-3)
                ).run();
                break;
            case RIGHT:
                poser.moveBy(
                        Distance.inInches(-3),
                        Distance.ZERO
                ).run();
                break;
        }
    }
}
