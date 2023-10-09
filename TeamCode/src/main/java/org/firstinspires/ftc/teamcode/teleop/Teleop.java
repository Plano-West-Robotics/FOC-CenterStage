package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

@TeleOp(name = "teleop")
public class Teleop extends OpMode {
    Hardware hardware;
    Drive drive;
    GamepadWrapper gp1;
    GamepadWrapper gp2;

    @Override
    public void init() {
        hardware = new Hardware(this);
        drive = new Drive(hardware, 0.7);
        drive.setFieldOriented(false);
        gp1 = new GamepadWrapper(gamepad1);
        gp2 = new GamepadWrapper(gamepad2);
    }

    @Override
    public void loop() {
        gp1.update(gamepad1);
        gp2.update(gamepad2);

        double y = gp1.getAnalogValue(Controls.GP1_STRAIGHT);
        double x = gp1.getAnalogValue(Controls.GP1_STRAFE);
        double turn = gp1.getAnalogValue(Controls.GP1_TURN);
        drive.drive(x, y, turn);
    }
}
