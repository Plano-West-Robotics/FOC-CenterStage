package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;

@TeleOp
public class IntakeTester extends OpModeWrapper {
    @Override
    public void run() {
        double power = gamepads.getAnalogValue(Gamepads.AnalogInput.GP1_LEFT_STICK_Y);
        hardware.intake.setPower(power);
        hardware.ramp.setPower(power);
    }
}
