package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.Macro;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.SimpleExecutor;
import org.firstinspires.ftc.teamcode.macro.Wait;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.Poser;
import org.firstinspires.ftc.teamcode.poser.Vector2;
import org.firstinspires.ftc.teamcode.poser.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.poser.localization.KalmanFilter;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.localization.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Box;
import org.firstinspires.ftc.teamcode.subsystems.ControlledArm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift3;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.openftc.apriltag.AprilTagDetectorJNI;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Lift3 lift;
    Box box;
    PlaneLauncher launcher;
    LED ledStrip;
    double driveSpeed;
    double intakeSpeed;
    boolean fieldOriented;
    SimpleExecutor motion = new SimpleExecutor();
    Poser poser;

    Macro launchMacro;
    Macro stackro;
    Macro pixelSwitchMacro;

    Localizer localizer;

    private static final double DEFAULT_SPEED = 312/435.;
    private static final double TURBO_SPEED = 1;
    private static final double SLOW_SPEED = 0.35;

    @Override
    public void setup() {
////        AprilTagDetector detector1 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
//        AprilTagDetector detector2 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
//        this.localizer = new KalmanFilter(
//                Pose.ZERO,
//                new ThreeDeadWheelLocalizer(hardware),
////                new AprilTagLocalizer(detector1, hardware, AprilTagLocalizer.Camera.FRONT),
//                new AprilTagLocalizer(detector2, hardware, AprilTagLocalizer.Camera.REAR)
//        );
//        FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);

        hardware.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveSpeed = 1;
        intakeSpeed = 1; // change this speed if you have to
        drive = new Drive(hardware, driveSpeed);
        drive.setFieldOriented(false);

        intake = new Intake(hardware, intakeSpeed);
        intake.stop();

        lift = new Lift3(hardware);

        box = new Box(hardware);

        launcher = new PlaneLauncher(hardware);
        launcher.idle().run();

        launchMacro = new Macro(
                Sequence.of(
                        launcher.aim(),
                        Wait.millis(500),
                        launcher.fire(),
                        Wait.millis(500),
                        launcher.idle()
                )
        );

        stackro = new Macro(
                Sequence.of(
                        Action.fromFn(() -> {
                            intake.setReversed(false);
                            intake.start();
                            intake.update();
                        }),
                        Wait.millis(500),
                        Action.fromFn(() -> {
                            intake.setReversed(true);
                            intake.update();
                        }),
                        Wait.millis(100),
                        Action.fromFn(() -> {
                            intake.setReversed(false);
                            intake.stop();
                            intake.update();
                        })
                )
        );

        pixelSwitchMacro = new Macro(
                Sequence.of(
                        Action.fromFn(() -> box.setFlapPosition(Box.FlapPosition.CLOSED)),
                        Wait.millis(300),
                        Action.fromFn(() -> box.setBlockerPosition(Box.BlockerPosition.UNBLOCK))
                )
        );

        ledStrip = new LED(hardware);
        ledStrip.setMode(LED.Mode.IDLE);
        ledStrip.update();

        poser = new Poser(hardware, DEFAULT_SPEED, false, Pose.ZERO);
        localizer = poser.localizer;
        FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        localizer.update();
        hardware.dashboardTelemetry.drawRobot(localizer.getPoseEstimate());
        hardware.log.addData("TeleOp", "pose", localizer.getPoseEstimate());

        telemetry.addData("Yaw", hardware.getYaw());

        if (gamepads.isPressed(Controls.TURBO_SPEED)) {
            driveSpeed = TURBO_SPEED;
        } else if (gamepads.isPressed(Controls.SLOW_SPEED)) {
            driveSpeed = SLOW_SPEED;
        } else {
            driveSpeed = DEFAULT_SPEED;
        }
        drive.setSpeed(driveSpeed);
        telemetry.addData("Drive speed", String.format("%.2f", drive.getSpeed()));

        if (!stackro.isRunning()) {
            if (gamepads.justPressed(Controls.TOGGLE_INTAKE) && !gamepads.isPressed(Gamepads.Button.GP1_START)) {
                intake.toggleRunning();
            }
            if (gamepads.justPressed(Controls.INTAKE_DIR_TOG)) {
                intake.reverse();
            }
        }

        if (gamepads.justPressed(Controls.STACKRO)) {
            stackro.start();
        }

        intakeSpeed = Range.clip(intakeSpeed, 0.1, 1.0);
        intake.setSpeed(intakeSpeed);
        telemetry.addData("Intake speed", String.format("%.2f", intakeSpeed));
        telemetry.addData("Intake reversed?", intake.isReversed());
        telemetry.addData("Intake running?", intake.isRunning());

        if (gamepads.justPressed(Controls.ARM_UP)) {
            lift.setArmPosition(ControlledArm.ArmPosition.UP);
        }
        if (gamepads.justPressed(Controls.ARM_DOWN)) {
            lift.setArmPosition(ControlledArm.ArmPosition.DOWN);
        }
        if (gamepads.justPressed(Controls.ARM_FIXEL)) {
            lift.setArmPosition(ControlledArm.ArmPosition.FIXEL);
        }

        if (!pixelSwitchMacro.isRunning()) {
            if (gamepads.justPressed(Controls.FLAP_OPEN)) {
                box.setFlapPosition(Box.FlapPosition.OPEN);
            }
            if (gamepads.justPressed(Controls.FLAP_CLOSED)) {
                box.setFlapPosition(Box.FlapPosition.CLOSED);
            }

            if (gamepads.justPressed(Controls.BLOCKER_OPEN)) {
                box.setBlockerPosition(Box.BlockerPosition.UNBLOCK);
            }
            if (gamepads.justPressed(Controls.BLOCKER_CLOSED)) {
                box.setBlockerPosition(Box.BlockerPosition.BLOCK);
            }
        }

        if (gamepads.justPressed(Controls.PIXEL_SWITCH)) {
            pixelSwitchMacro.start();
        }

        // press to aim and fire
        if (!launchMacro.isRunning()) {
            if (gamepads.justPressed(Controls.LAUNCH_PLANE)) {
                launchMacro.start();
            }
        }

        if (gamepads.justPressed(Controls.FIELD_ORIENTED)) {
//            drive.toggleFieldOriented();
            fieldOriented = !fieldOriented;
        }

//        if (gamepads.justPressed(Controls.RESET_IMU)) {
//            drive.resetYaw();
//        }

        // proof-of-concept
        if (gamepads.justPressed(Gamepads.Button.GP1_RIGHT_TRIGGER)) {
            if (localizer.getPoseEstimate().pos.y.valInMM() < 0) {
                motion.run(poser.goTo(
                        Distance.inTiles(2).add(Distance.inInches(3)),
                        Distance.inTiles(-1.5),
                        Angle.BACKWARD
                ));
            } else {
                motion.run(poser.goTo(
                        Distance.inTiles(2).add(Distance.inInches(3)),
                        Distance.inTiles(1.5),
                        Angle.BACKWARD
                ));
            }
        }
        //as

        double x = gamepads.getAnalogValue(Controls.STRAIGHT);
        double y = -gamepads.getAnalogValue(Controls.STRAFE);
        double turn = -gamepads.getAnalogValue(Controls.TURN);
        if (x != 0 || y != 0 || turn != 0) motion.stop();
        if (!motion.isRunning()) {
            Vector2 xy = new Vector2(x, y);
            if (fieldOriented) {
                xy = xy.rot(localizer.getPoseEstimate().yaw);
            }
            drive.drive(xy.x, xy.y, turn);
        }

//        telemetry.addData("Field oriented enabled", drive.getFieldOriented());
        telemetry.addData("Field oriented enabled", fieldOriented);

        lift.setPower(gamepads.getAnalogValue(Controls.LIFT) * (gamepads.isPressed(Controls.LIFT_SLOW) ? 0.35 : 1));
        if (gamepads.isPressed(Controls.LIFT_GO_TO_HANG)) {
            lift.setArmPosition(ControlledArm.ArmPosition.UP);
            lift.setTarget(Lift3.HANG_HEIGHT);
        }
        if (gamepads.isPressed(Controls.LIFT_GO_TO_BOTTOM)) {
            lift.setTarget(0);
        }
        if (gamepads.justPressed(Controls.LIFT_HANG)) {
            lift.hang();
        }

        telemetry.addData("Left Lift Encoder", hardware.liftL.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", hardware.liftR.getCurrentPosition());
        telemetry.addData("Lift Position", lift.getPos());

        if (lift.getArmPosition() == ControlledArm.ArmPosition.FIXEL) {
            ledStrip.setMode(LED.Mode.FIXEL_OVERRIDE);
        } else if (intake.isReversed()) {
            ledStrip.setMode(LED.Mode.EJECT_OVERRIDE);
        } else {
            ledStrip.setMode(LED.Mode.RUNNING);
        }

        ledStrip.update();
        telemetry.addData("Top state", ledStrip.sensor.top_state);
        telemetry.addData("Bottom state", ledStrip.sensor.bottom_state);

        telemetry.addData("Magnet sensed", hardware.slideLimitSwitch.isPressed());
        telemetry.addData("arm position", lift.getArmPosition());

        intake.update();
        lift.update(gamepads.isPressed(Controls.LIFT_OVERRIDE));

        launchMacro.update();
        stackro.update();
        pixelSwitchMacro.update();
        motion.update();
    }
}
