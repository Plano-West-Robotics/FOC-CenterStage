package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Arm arm;
    PlaneLauncher launcher;
    double driveSpeed;
    double intakeSpeed;

    @Override
    public void setup() {
        driveSpeed = 0.7;
        intakeSpeed = 1; // change this speed if you have to
        drive = new Drive(hardware, driveSpeed);
        drive.setFieldOriented(false);

        intake = new Intake(hardware, intakeSpeed);

        arm = new Arm(hardware, telemetry);
        launcher = new PlaneLauncher(hardware);
    }

    @Override
    public void start() { // runs once before loop()
        intake.start();
    }

    @Override
    public void run() {
        telemetry.addData("Yaw", hardware.getYaw());

        if (gamepads.justPressed(Controls.DRIVE_FASTER)) {
            driveSpeed += 0.15;
        }
        if (gamepads.justPressed(Controls.DRIVE_SLOWER)) {
            driveSpeed -= 0.15;
        }
        driveSpeed = Range.clip(driveSpeed, 0.1, 1.0);
        drive.setSpeed(driveSpeed);
        telemetry.addData("Drive speed", driveSpeed);

        if (gamepads.justPressed(Controls.TOGGLE_INTAKE)) {
            intake.toggleRunning();
        }
        if (gamepads.justPressed(Controls.INTAKE_FASTER)) {
            intakeSpeed += 0.15;
        }
        if (gamepads.justPressed(Controls.INTAKE_SLOWER)) {
            intakeSpeed -= 0.15;
        }
        if (gamepads.justPressed(Controls.INTAKE_REVERSE)) {
            intake.reverse();
        }
        intakeSpeed = Range.clip(intakeSpeed, 0.1, 1.0);
        intake.setSpeed(intakeSpeed);
        telemetry.addData("Intake speed", intakeSpeed);
        telemetry.addData("Intake reversed?", intake.isReversed());

        if (gamepads.justPressed(Controls.ARM_UP)) {
            arm.setArmPosition(Arm.ArmPosition.UP);
        }
        if (gamepads.justPressed(Controls.ARM_DOWN)) {
            arm.setArmPosition(Arm.ArmPosition.DOWN);
        }

        if (gamepads.justPressed(Controls.PEG_ENGAGE)) {
            arm.setPegPosition(Arm.PegPosition.ENGAGED);
        }
        if (gamepads.justPressed(Controls.PEG_DISENGAGE)) {
            arm.setPegPosition(Arm.PegPosition.DISENGAGED);
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
        intake.update();
    }
}
