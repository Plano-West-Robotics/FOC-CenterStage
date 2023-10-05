package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        drive = new Drive(hardware);
//        drive.setSpeed(0.5);
        drive.setFieldOriented(false);

        gp1 = new GamepadWrapper();
        gp2 = new GamepadWrapper();

        gp1.update(gamepad1);
        gp2.update(gamepad2);
    }

    @Override
    public void loop() {
        gp1.update(gamepad1);
        gp2.update(gamepad2);

        double x = gp1.getAnalogValue(GamepadWrapper.Button.LEFT_STICK_X);
        double y = gp1.getAnalogValue(GamepadWrapper.Button.LEFT_STICK_Y);
        double turn = gp1.getAnalogValue(GamepadWrapper.Button.RIGHT_STICK_X);
        drive.drive(x, y, turn);
    }
}
