package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

@TeleOp
public class IntakeTester extends OpModeWrapper {
    @Override
    public void run() {
        double power = gamepad1.getAnalogValue(GamepadWrapper.AnalogInput.LEFT_STICK_Y);
        hardware.intake.setPower(power);
        hardware.ramp.setPower(power);
    }
}
