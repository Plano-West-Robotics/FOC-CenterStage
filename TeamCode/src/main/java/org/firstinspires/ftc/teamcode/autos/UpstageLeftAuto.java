package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.inchworm.InchWormBackup;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ControlledLift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@Autonomous(name = "Upstage Left Auto")
public class UpstageLeftAuto extends LinearOpMode {
    final static double TILE = 23.625;

    // chosen by fair dice roll
    // guaranteed to be random
    public static FreeSightPipeline.Side KARXBOX_LOCATION = FreeSightPipeline.Side.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(this);
        Intake intake = new Intake(hardware, 0.5);
        Arm arm = new Arm(hardware, telemetry);
        arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        arm.holdElbows();
        ControlledLift lift = new ControlledLift(hardware);
        InchWormBackup inchWorm = new InchWormBackup(this);
        inchWorm.tracker.setPoseEstimate(new InchWormBackup.Pose(
                0.5 * TILE + 0.5 * (TILE - 0.75 - 18),
                -(2.5 * TILE + 0.5 * (TILE - 0.75 - 18)),
                Math.PI / 2
        ));

        waitForStart();

        double yOffsetAtBackdrop = 0;
        switch (KARXBOX_LOCATION) {
            case LEFT:
                yOffsetAtBackdrop = 8;
                inchWorm.moveTo(
                        0.5 * TILE,
                        -1.5 * TILE,
                        Math.PI
                );
                break;
            case MIDDLE:
                yOffsetAtBackdrop = 0;
                inchWorm.moveTo(
                        0.5 * TILE + 7,
                        -1.5 * TILE
                );
                break;
            case RIGHT:
                yOffsetAtBackdrop = -8;
                inchWorm.moveTo(
                        TILE - 1,
                        -2 * TILE + 2.5
                );
                break;
        }

        intake.reverse();
        intake.start();
        intake.update();
        sleep(500);
        intake.stop();
        intake.reverse();
        intake.update();

        // arm up
        arm.setArmPosition(Arm.ArmPosition.UP);
        // go in front of backdrop
        inchWorm.moveTo(
                2 * TILE,
                -1.5 * TILE + yOffsetAtBackdrop,
                Math.PI
        );
        // lift up
        lift.setTarget(0.20);
        while (lift.update());
        lift.stop();
        // move closer to backdrop
        inchWorm.moveTo(
                2 * TILE + 5,
                -1.5 * TILE + yOffsetAtBackdrop
        );
        // drop pixel
        arm.setFlapPosition(Arm.FlapPosition.OPEN);
        sleep(1000);
        arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        // move over
        inchWorm.moveTo(
                2 * TILE,
                -2.5 * TILE
        );
        // lower slide and move arm in
        lift.setTarget(0.03);
        while (lift.update());
        lift.stop();
        arm.setArmPosition(Arm.ArmPosition.DOWN);
        sleep(500);
        arm.setFlapPosition(Arm.FlapPosition.OPEN);
        // move to final parking position
        inchWorm.moveTo(
                2.5 * TILE,
                -2.5 * TILE
        );
    }
}
