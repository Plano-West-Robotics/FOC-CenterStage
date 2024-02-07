package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="Calibrate Camera Intrinsics")
public class CameraCalibration extends OpModeWrapper {
    private CalibrationPipeline pipeline;

    @Override
    public void setup() {
        pipeline = new CalibrationPipeline();
        hardware.frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                hardware.rearCam.setPipeline(pipeline);
                hardware.rearCam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(hardware.rearCam, 0);
            }

            @Override
            public void onError(int errorCode) {}
        });
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
