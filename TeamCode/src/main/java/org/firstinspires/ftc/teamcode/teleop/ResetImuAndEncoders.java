package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class ResetImuAndEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        for (DcMotor motor : hardwareMap.dcMotor) {
            DcMotor.RunMode oldMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(oldMode);
        }

        for (IMU imu : hardwareMap.getAll(IMU.class)) {
            imu.resetYaw();
        }
    }
}
