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
import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.ControlledArm;
import org.firstinspires.ftc.teamcode.subsystems.ControlledArmOld;
import org.firstinspires.ftc.teamcode.subsystems.ControlledLift;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public abstract class AutoBase extends LinearOpMode {
    Alliance alliance;
    boolean isFlipped;

    Hardware hardware;
    Intake intake;
    Box box;
    ControlledArmOld arm;
    ControlledLift lift;
    Poser poser;

    public enum UpOrDownStage {
        UPSTAGE, DOWNSTAGE
    }

    public void setup(Alliance alliance, UpOrDownStage location) {
        this.alliance = alliance;
        this.isFlipped = alliance.isBlue();

        this.hardware = new Hardware(this);
        this.intake = new Intake(hardware, 0.3);
        this.box = new Box(hardware);
        this.arm = new ControlledArmOld(hardware);
        this.lift = new ControlledLift(hardware);
        box.setFlapPosition(Box.FlapPosition.CLOSED);

        // i could have sworn we did this somewhere else but ig not
        PlaneLauncher launcher = new PlaneLauncher(hardware);
        launcher.idle().run();


        // starting pose calculations

        // starting tile
        double startingX = 0;
        switch (location) {
            case UPSTAGE:
                startingX = 0.5;
                break;
            case DOWNSTAGE:
                startingX = -1.5;
                break;
            default:
                throw new RuntimeException();
        }
        Pose initialTile = new Pose(
                Distance2.inTiles(startingX, -2.5),
                Angle.LEFT
        );
        if (isFlipped) initialTile = initialTile.flippedAcrossXAxis();

        // position within that tile
        Pose initialPose = initialTile.then(new Pose(
                new Distance2(
                        // in the ROBOT's coordinate scheme (b/c .then)
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(17)).neg(),
                        Distance.ONE_TILE_WITHOUT_BORDER.sub(Distance.inInches(18)).neg()
                ).div(2)
                .add(new Distance2(
                        Distance.inInches(1),
                        Distance.ZERO
                )),
                Angle.ZERO
        ));

        boolean willInterfereWithTruss =
                (alliance.isBlue() && location == UpOrDownStage.UPSTAGE)
                || (alliance.isRed() && location == UpOrDownStage.DOWNSTAGE);
        if (willInterfereWithTruss) {
            initialPose = initialPose.then(new Pose(
                    new Distance2(
                            Distance.ZERO,
                            Distance.inInches(2)
                    ),
                    Angle.ZERO
            ));
        }

        this.poser = new Poser(hardware, 0.9, isFlipped, initialPose);
    }

    public FreeSightPipeline.Side runVisionUntilStart() {
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

        return randomization;
    }
}
