package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class DashboardServoTuner extends OpMode {
    @Override
    public void init() {
        FtcDashboard db = FtcDashboard.getInstance();

        for (ServoImplEx s : hardwareMap.getAll(ServoImplEx.class)) {
            String name = hardwareMap.getNamesOf(s).iterator().next();
            if (name == null) continue;

            db.addConfigVariable(this.getClass().getSimpleName(), name, new ValueProvider<String>() {
                final ServoImplEx servo = s;
                double pos = 0.5;
                String verbatimInput = "";
                boolean isActive = false;

                @Override
                public String get() {
                    return verbatimInput;
                }

                @Override
                public void set(String value) {
                    if (value.equals("")) {
                        this.isActive = false;
                        this.servo.setPwmDisable();
                        this.verbatimInput = value;
                        return;
                    }

                    try {
                        this.pos = Double.parseDouble(value);
                    } catch (NumberFormatException e) {
                        // do not acknowledge the request
                        return;
                    }

                    this.isActive = true;
                    this.servo.setPwmEnable();
                    this.servo.setPosition(this.pos);
                    this.verbatimInput = value;
                }
            });
        }
    }

    @Override
    public void loop() {}
}
