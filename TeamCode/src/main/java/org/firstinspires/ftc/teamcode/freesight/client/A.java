package org.firstinspires.ftc.teamcode.freesight.client;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.pipelines.APipeline;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FSTuner;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class A {
    public APipeline pipe;
    private final OpenCvCamera web;
    private final Telemetry telemetry;

    public A(Hardware hardware, Telemetry telemetry) {
        web = hardware.frontCam;
        this.telemetry = telemetry;
    }

    public void init() {
        pipe = new APipeline();
        web.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                web.setPipeline(pipe);
                web.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(web, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error:", "Camera produced error code: " + errorCode);
                stop();
            }
        });
    }

    public double getPosition() {
        return pipe.positionState;
    }

    public void stop() {
        FtcDashboard.getInstance().stopCameraStream();
        web.closeCameraDeviceAsync(() -> {
            pipe.releaseFrames();
        });
    }
}
