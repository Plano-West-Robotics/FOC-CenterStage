package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="test")
public class MotorTester extends OpMode {
    DcMotor intake;
    DcMotor ramp;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        ramp = hardwareMap.get(DcMotor.class, "ramp");
    }
    @Override
    public void loop() {
        intake.setPower(-gamepad1.left_stick_y);
        ramp.setPower(-gamepad1.left_stick_y * 0.25);
    }
}
