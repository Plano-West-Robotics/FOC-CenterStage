package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "CCCCCCCCC")
public class Teleop extends OpModeWrapper {
    Drive drive;

    @Override
    public void setup() {
        drive = new Drive(hardware, 0.7);
        drive.setFieldOriented(false);
    }

    @Override
    public void run() {
        double y = gamepad1.getAnalogValue(Controls.GP1_STRAIGHT);
        double x = gamepad1.getAnalogValue(Controls.GP1_STRAFE);
        double turn = gamepad1.getAnalogValue(Controls.GP1_TURN);
        drive.drive(x, y, turn);
    }
}
