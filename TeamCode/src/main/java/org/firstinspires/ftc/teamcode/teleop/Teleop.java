package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;
    double speed;

    @Override
    public void setup() {
        speed = 0.7;
        drive = new Drive(hardware, speed);
        drive.setFieldOriented(false);
    }

    @Override
    public void run() {
        telemetry.addData("Yaw", hardware.getYaw());

        if (gamepad1.justPressed(Controls.GP1_FASTER)) {
            speed += 0.15;
        }
        if (gamepad1.justPressed(Controls.GP1_SLOWER)) {
            speed -= 0.15;
        }
        speed = Range.clip(speed, 0.1, 1.0);
        drive.setSpeed(speed);
        telemetry.addData("Speed", speed);

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
    }
}
