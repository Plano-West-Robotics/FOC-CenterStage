package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

@TeleOp(group="test")
public class MotorTester extends OpMode {
    Hardware hardware;
    GamepadWrapper gp1;

    @Override
    public void init() {
        hardware = new Hardware(this);
        gp1 = new GamepadWrapper(gamepad1);
    }

    @Override
    public void loop() {
        gp1.update(gamepad1);

        hardware.intake.setPower(gp1.getAnalogValue(GamepadWrapper.AnalogInput.LEFT_STICK_Y));
    }
}
