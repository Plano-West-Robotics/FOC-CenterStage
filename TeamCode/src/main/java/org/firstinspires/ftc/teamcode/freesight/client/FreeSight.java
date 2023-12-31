package org.firstinspires.ftc.teamcode.freesight.client;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FSTuner;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.openftc.easyopencv.*;

public class FreeSight {
    public FreeSightPipeline pipe;
    private final OpenCvWebcam web;
    private final Telemetry telemetry;
    private boolean debug = false;

    public FreeSight(Hardware hardware, Telemetry telemetry) {
        web = (OpenCvWebcam) hardware.webcam;
        this.telemetry = telemetry;
    }
    public FreeSight(Hardware hardware, Telemetry telemetry, boolean debug) {
        web = (OpenCvWebcam) hardware.webcam;
        this.telemetry = telemetry;
        this.debug = debug;
    }


    public void init() {
        if(!debug)
            pipe = new FreeSightPipeline();
        else
            pipe = new FSTuner();
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

    public FreeSightPipeline.Side getPosition() {
        return pipe.positionState;
    }

    public void stop() {
        FtcDashboard.getInstance().stopCameraStream();
        web.closeCameraDeviceAsync(() -> {
            pipe.releaseFrames();
        });
    }
}
