package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Sensor;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Lift lift;
    Arm arm;
    PlaneLauncher launcher;
    LED ledStrip;
    double driveSpeed;
    double intakeSpeed;

    @Override
    public void setup() {
        driveSpeed = 1;
        intakeSpeed = 1; // change this speed if you have to
        drive = new Drive(hardware, driveSpeed);
        drive.setFieldOriented(false);

        intake = new Intake(hardware, intakeSpeed);
        intake.stop();

        lift = new Lift(hardware);

        arm = new Arm(hardware, telemetry);

        launcher = new PlaneLauncher(hardware);
        launcher.idle();

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
        telemetry.addData("Yaw", hardware.getYaw());

        if (gamepads.isPressed(Controls.SLOW_SPEED)) {
            driveSpeed = 0.7;
        } else if (gamepads.isPressed(Controls.SUPER_SLOW_SPEED)) {
            driveSpeed = 0.35;
        } else {
            driveSpeed = 1;
        }
        driveSpeed = Range.clip(driveSpeed, 0.1, 1.0);
        drive.setSpeed(driveSpeed);
        telemetry.addData("Drive speed", String.format("%.2f", drive.getSpeed()));

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
        intakeSpeed = Range.clip(intakeSpeed, 0.1, 1.0);
        intake.setSpeed(intakeSpeed);
        telemetry.addData("Intake speed", String.format("%.2f", intakeSpeed));
        telemetry.addData("Intake reversed?", intake.isReversed());
        telemetry.addData("Intake running?", intake.isRunning());

        if (gamepads.justPressed(Controls.ARM_UP)) {
            arm.setArmPosition(Arm.ArmPosition.UP);
        }
        if (gamepads.justPressed(Controls.ARM_DOWN)) {
            arm.setArmPosition(Arm.ArmPosition.DOWN);
        }

        if (gamepads.justPressed(Controls.FLAP_OPEN)) {
            arm.setFlapPosition(Arm.FlapPosition.OPEN);
        }
        if (gamepads.justPressed(Controls.FLAP_CLOSED)) {
            arm.setFlapPosition(Arm.FlapPosition.CLOSED);
        }

        if (gamepads.justPressed(Controls.BLOCKER_OPEN)) {
            arm.setBlockerPosition(Arm.BlockerPosition.UNBLOCK);
        }
        if (gamepads.justPressed(Controls.BLOCKER_CLOSED)) {
            arm.setBlockerPosition(Arm.BlockerPosition.BLOCK);
        }

        // press to aim and fire
        if (gamepads.justPressed(Controls.LAUNCH_PLANE)) {
            launcher.aim();
            launcher.fire();
        }

        double y = gamepads.getAnalogValue(Controls.STRAIGHT);
        double x = gamepads.getAnalogValue(Controls.STRAFE);
        double turn = gamepads.getAnalogValue(Controls.TURN);
        drive.drive(x, y, turn);

        lift.run(gamepads.getAnalogValue(Controls.LIFT));

        telemetry.addData("Left Lift Encoder", hardware.liftL.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", hardware.liftR.getCurrentPosition());

        ledStrip.update();
        telemetry.addData("Top state", ledStrip.sensor.top_state);
        telemetry.addData("Bottom state", ledStrip.sensor.bottom_state);

        intake.update();
    }
}
