package org.firstinspires.ftc.teamcode.apriltag;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;

@TeleOp(name="Calibrate Camera Intrinsics")
public class CameraCalibration extends OpModeWrapper {
    private CalibrationPipeline pipeline;

    @Override
    public void setup() {
        pipeline = new CalibrationPipeline();
        hardware.openCamera(pipeline);
    }

    @Override
    public void run() {
        if (gamepads.justPressed(Gamepads.Button.GP1_CROSS)) {
            pipeline.onViewportTapped();
        }

        this.telemetry.addData("Number of data points", pipeline.dataPointCount);
        this.telemetry.addData("Enough data points for estimate?", pipeline.hasEstimateBeenMade);
        if (pipeline.hasEstimateBeenMade) {
            this.telemetry.addData("Estimate error", pipeline.estimateError);
            this.telemetry.addData("fx", pipeline.estimatedIntrinsics.fx);
            this.telemetry.addData("fy", pipeline.estimatedIntrinsics.fy);
            this.telemetry.addData("cx", pipeline.estimatedIntrinsics.cx);
            this.telemetry.addData("cy", pipeline.estimatedIntrinsics.cy);
        }
        this.telemetry.update();
    }
}
