package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;

@TeleOp
public class WhatsTheVoltage extends OpModeWrapper {
    public void run() {
        telemetry.addData("Voltage (V)", hardware.voltageSensor.getVoltage());
    }
}
