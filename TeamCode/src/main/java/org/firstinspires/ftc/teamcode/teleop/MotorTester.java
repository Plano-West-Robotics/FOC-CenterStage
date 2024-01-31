package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="test")
public class MotorTester extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "temp");
    }
    @Override
    public void loop() {
        motor.setPower(-gamepad1.left_stick_y);
    }
}
