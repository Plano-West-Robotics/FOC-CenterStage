package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class DashboardServoTuner extends OpMode {
    public static double servoPos = 0.5;

    ServoImplEx servo = null;
    String servoName = "";

    @Override
    public void init() {
        FtcDashboard db = FtcDashboard.getInstance();

        Telemetry telemetry = db.getTelemetry();
        telemetry.addLine("The following servos are available:");
        for (String name : hardwareMap.getAllNames(ServoImplEx.class)) {
            telemetry.addLine(name);
        }
        telemetry.update();

        db.addConfigVariable(this.getClass().getSimpleName(), "servoName", new ValueProvider<String>() {
            @Override
            public String get() {
                return servoName;
            }

            @Override
            public void set(String value) {
                if (hardwareMap.getNamesOf(servo).contains(value)) return;

                ServoImplEx oldServo = servo;

                if (value.equals("")) {
                    servo = null;
                } else {
                    // "you should ONLY call this method during the Init phase of your OpMode" 🤓
                    ServoImplEx maybeServo = hardwareMap.tryGet(ServoImplEx.class, value);
                    if (maybeServo == null) return;
                    servo = maybeServo;
                }
                servoName = value;

                if (oldServo != null) {
                    oldServo.setPwmDisable();
                }
            }
        });
    }

    @Override
    public void loop() {
        if (servo != null) {
            servo.setPosition(servoPos);
        }
    }
}
