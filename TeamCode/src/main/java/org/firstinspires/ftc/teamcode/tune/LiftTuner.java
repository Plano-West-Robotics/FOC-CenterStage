package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;

@TeleOp(group = "tune")
public class LiftTuner extends OpModeWrapper {
    @Override
    public void setup() {
        this.hardware.liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.hardware.liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.hardware.liftL.setPower(0);
        this.hardware.liftR.setPower(0);
    }

    @Override
    public void run() {
        if (gamepads.isPressed(Gamepads.Button.GP1_CROSS)) {
            this.hardware.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hardware.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.hardware.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.hardware.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("liftLPos", this.hardware.liftL.getCurrentPosition());
        telemetry.addData("liftRPos", this.hardware.liftR.getCurrentPosition());
    }
}
