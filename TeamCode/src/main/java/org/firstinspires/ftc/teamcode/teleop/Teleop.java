package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.Macro;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.poser.localization.KalmanFilter;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.localization.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ControlledArm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.openftc.apriltag.AprilTagDetectorJNI;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Lift lift;
    ControlledArm arm;
    PlaneLauncher launcher;
    LED ledStrip;
    double driveSpeed;
    double intakeSpeed;
    boolean armManual;

    Macro launchMacro;
    Macro stackro;
    Macro pixelSwitchMacro;

    Localizer localizer;

    private static final double DEFAULT_SPEED = 312/435.;
    private static final double TURBO_SPEED = 1;
    private static final double SLOW_SPEED = 0.35;

    @Override
    public void setup() {
        AprilTagDetector detector1 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        AprilTagDetector detector2 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        this.localizer = new KalmanFilter(
                Pose.ZERO,
                new TwoDeadWheelLocalizer(hardware),
                new AprilTagLocalizer(detector1, hardware, AprilTagLocalizer.Camera.FRONT),
                new AprilTagLocalizer(detector2, hardware, AprilTagLocalizer.Camera.REAR)
        );
        FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);

        driveSpeed = 1;
        intakeSpeed = 1; // change this speed if you have to
        drive = new Drive(hardware, driveSpeed);
        drive.setFieldOriented(true);

        intake = new Intake(hardware, intakeSpeed);
        intake.stop();

        lift = new Lift(hardware);

        arm = new ControlledArm(hardware);
        armManual = false;

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
                        Action.fromFn(() -> intake.setReversed(false)),
                        Action.fromFn(() -> intake.start()),
                        Action.fromFn(() -> intake.update()),
                        Wait.millis(100),
                        Action.fromFn(() -> intake.stop()),
                        Action.fromFn(() -> intake.update())
                )
        );

        pixelSwitchMacro = new Macro(
                Sequence.of(
                        Action.fromFn(() -> arm.arm.setBlockerPosition(Arm.BlockerPosition.BLOCK)),
                        Wait.millis(100),
                        Action.fromFn(() -> arm.arm.setFlapPosition(Arm.FlapPosition.CLOSED)),
                        Wait.millis(300),
                        Action.fromFn(() -> arm.arm.setFlapPosition(Arm.FlapPosition.OPEN))
                )
        );

        ledStrip = new LED(hardware);
        ledStrip.setMode(LED.Mode.IDLE);
        ledStrip.update();
    }

    @Override
    public void start() {
        ledStrip.setMode(LED.Mode.RUNNING);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        localizer.update();
        hardware.dashboardTelemetry.drawRobot(localizer.getPoseEstimate());

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
            if (gamepads.justPressed(Controls.TOGGLE_INTAKE)) {
                intake.toggleRunning();
            }
            if (gamepads.justPressed(Controls.INTAKE_DIR_TOG)) {
                intake.reverse();
                if (intake.isReversed()) {
                    ledStrip.setMode(LED.Mode.EJECT_OVERRIDE);
                } else {
                    ledStrip.setMode(LED.Mode.RUNNING);
                }
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

        if (gamepads.isPressed(Controls.ARM_BACK_TO_AUTO)) {
            armManual = false;
        }
        if (gamepads.isPressed(Controls.ARM_UP) || gamepads.isPressed(Controls.ARM_DOWN)) {
            armManual = true;
        }
        if (armManual) {
            if (gamepads.justPressed(Controls.ARM_UP)) {
                arm.moveUp();
            }
            if (gamepads.justPressed(Controls.ARM_DOWN)) {
                arm.moveDown();
            }
        } else {
            if (hardware.slideLimitSwitch.isPressed()) {
                arm.moveDown();
            } else {
                arm.moveUp();
            }
        }

        if (!pixelSwitchMacro.isRunning()) {
            if (gamepads.justPressed(Controls.FLAP_OPEN)) {
                arm.arm.setFlapPosition(Arm.FlapPosition.OPEN);
            }
            if (gamepads.justPressed(Controls.FLAP_CLOSED)) {
                arm.arm.setFlapPosition(Arm.FlapPosition.CLOSED);
            }

            if (gamepads.justPressed(Controls.BLOCKER_OPEN)) {
                arm.arm.setBlockerPosition(Arm.BlockerPosition.UNBLOCK);
            }
            if (gamepads.justPressed(Controls.BLOCKER_CLOSED)) {
                arm.arm.setBlockerPosition(Arm.BlockerPosition.BLOCK);
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
            drive.toggleFieldOriented();
        }

        if (gamepads.justPressed(Controls.RESET_IMU)) {
            drive.resetYaw();
        }

        double y = gamepads.getAnalogValue(Controls.STRAIGHT);
        double x = gamepads.getAnalogValue(Controls.STRAFE);
        double turn = gamepads.getAnalogValue(Controls.TURN);
        drive.drive(x, y, turn);

        telemetry.addData("Field oriented enabled", drive.getFieldOriented());

        lift.run(gamepads.getAnalogValue(Controls.LIFT));

        telemetry.addData("Left Lift Encoder", hardware.liftL.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", hardware.liftR.getCurrentPosition());

        ledStrip.update();
        telemetry.addData("Top state", ledStrip.sensor.top_state);
        telemetry.addData("Bottom state", ledStrip.sensor.bottom_state);

        telemetry.addData("Magnet sensed", hardware.slideLimitSwitch.isPressed());
        telemetry.addData("Auto arm state", armManual ? "manual" : "auto");

        intake.update();
        arm.update();

        launchMacro.update();
        stackro.update();
        pixelSwitchMacro.update();
    }
}
