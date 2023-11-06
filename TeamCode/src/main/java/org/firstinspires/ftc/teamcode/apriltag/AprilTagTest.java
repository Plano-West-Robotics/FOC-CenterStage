package org.firstinspires.ftc.teamcode.apriltag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.teleop.Controls;

@TeleOp(name="AprilTag Test")
public class AprilTagTest extends OpModeWrapper {
    private AprilTagPipeline pipeline;
    private Drive drive;

    @Override
    public void setup() {
        pipeline = new AprilTagPipeline();
        hardware.openCamera(pipeline);
        drive = new Drive(hardware, 0.3);
    }

    @Override
    public void run() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("blue");

        for (int i = 0; i < pipeline.cameraPoseEstimates.length; i++) {
            if (pipeline.isVisible[i]) {
                double[] xyz = pipeline.cameraPoseEstimates[i].tvec.toArray();
                packet.fieldOverlay().fillCircle(xyz[0] / 25.4, xyz[1] / 25.4, 1);
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        drive.drive(
                gamepads.getAnalogValue(Controls.STRAFE),
                gamepads.getAnalogValue(Controls.STRAIGHT),
                gamepads.getAnalogValue(Controls.TURN)
        );
    }
}
