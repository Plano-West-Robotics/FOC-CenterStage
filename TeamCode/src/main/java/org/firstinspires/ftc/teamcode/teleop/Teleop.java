package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Lift lift;
    Arm arm;
    PlaneLauncher launcher;
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

        if (gamepads.justPressed(Controls.FLAP_TOGGLE)) {
            arm.toggleFlapPosition();
        }

        if (gamepads.justPressed(Controls.BLOCKER_TOGGLE)) {
            arm.toggleBlockerPosition();
        }

        if (gamepads.justPressed(Controls.LAUNCH_PLANE)) {
            launcher.disengage();
        }

        if (gamepads.justPressed(Controls.FIELD_ORIENTED)) {
            drive.toggleFieldOriented();
        }
        if (gamepads.justPressed(Controls.RESET_IMU)) {
            hardware.resetYaw();
        }
        telemetry.addData("Field oriented enabled", drive.getFieldOriented());

        double y = gamepads.getAnalogValue(Controls.STRAIGHT);
        double x = gamepads.getAnalogValue(Controls.STRAFE);
        double turn = gamepads.getAnalogValue(Controls.TURN);
        drive.drive(x, y, turn);

        lift.run(gamepads.getAnalogValue(Controls.LIFT));

        telemetry.addData("Left Lift Encoder", hardware.liftL.getCurrentPosition());
        telemetry.addData("Right Lift Encoder", hardware.liftR.getCurrentPosition());

        intake.update();
    }
}
