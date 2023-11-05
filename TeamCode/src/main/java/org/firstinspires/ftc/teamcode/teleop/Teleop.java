package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    Intake intake;
    Arm arm;
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
    }

    @Override
    public void start() { // runs once before loop()
        intake.start();
    }

    @Override
    public void run() {
        telemetry.addData("Yaw", hardware.getYaw());

        if (gamepad1.justPressed(Controls.GP1_FASTER)) {
            driveSpeed += 0.15;
        }
        if (gamepad1.justPressed(Controls.GP1_SLOWER)) {
            driveSpeed -= 0.15;
        }
        driveSpeed = Range.clip(driveSpeed, 0.1, 1.0);
        drive.setSpeed(driveSpeed);
        telemetry.addData("Drive speed", driveSpeed);

        if (gamepad2.justPressed(Controls.GP2_TOGGLE_INTAKE)) {
            intake.toggleRunning();
        }
        if (gamepad2.justPressed(Controls.GP2_INTAKE_SPEED_UP)) {
            intakeSpeed += 0.15;
        }
        if (gamepad2.justPressed(Controls.GP2_INTAKE_SPEED_DOWN)) {
            intakeSpeed -= 0.15;
        }
        if (gamepad2.justPressed(Controls.GP2_INTAKE_REVERSE)) {
            intake.reverse();
        }
        intakeSpeed = Range.clip(intakeSpeed, 0.1, 1.0);
        intake.setSpeed(intakeSpeed);
        telemetry.addData("Intake speed", intakeSpeed);
        telemetry.addData("Intake reversed?", intake.isReversed());

        if (gamepad2.justPressed(Controls.GP2_ARM_UP)) {
            arm.setArmPosition(Arm.ArmPosition.UP);
        }
        if (gamepad2.justPressed(Controls.GP2_ARM_DOWN)) {
            arm.setArmPosition(Arm.ArmPosition.DOWN);
        }

        if (gamepad2.justPressed(Controls.GP2_PEG_ENGAGE)) {
            arm.setPegPosition(Arm.PegPosition.ENGAGED);
        }
        if (gamepad2.justPressed(Controls.GP2_PEG_DISENGAGE)) {
            arm.setPegPosition(Arm.PegPosition.DISENGAGED);
        }

        if (gamepad1.justPressed(Controls.GP1_FIELD_ORIENTED)) {
            drive.toggleFieldOriented();
        }
        if (gamepad1.justPressed(Controls.GP1_RESET_IMU)) {
            hardware.resetYaw();
        }
        telemetry.addData("Field oriented enabled", drive.getFieldOriented());

        double y = gamepad1.getAnalogValue(Controls.GP1_STRAIGHT);
        double x = gamepad1.getAnalogValue(Controls.GP1_STRAFE);
        double turn = gamepad1.getAnalogValue(Controls.GP1_TURN);
        drive.drive(x, y, turn);
        intake.update();
    }
}
