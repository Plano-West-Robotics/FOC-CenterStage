package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

@TeleOp(group="test")
public class MotorTester extends OpModeWrapper {
    @Override
    public void run() {
        hardware.intake.setPower(gamepad1.getAnalogValue(GamepadWrapper.AnalogInput.LEFT_STICK_Y));
    }
}
