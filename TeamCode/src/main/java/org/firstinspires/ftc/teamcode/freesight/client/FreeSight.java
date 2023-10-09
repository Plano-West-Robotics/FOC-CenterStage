package org.firstinspires.ftc.teamcode.freesight.client;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.openftc.easyopencv.*;

public class FreeSight {
    private FreeSightPipeline pipe;
    private final OpenCvWebcam web;
    private final Telemetry telemetry;

    public FreeSight(OpMode op) {
        web = OpenCvCameraFactory.getInstance().createWebcam(
                op.hardwareMap.get(WebcamName.class, "Webcam 1")
        );
        telemetry = op.telemetry;
    }

    public void init() {
        pipe = new FreeSightPipeline();

        web.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                web.setPipeline(pipe);
                web.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
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
        web.stopStreaming();
    }
}
