package org.firstinspires.ftc.teamcode.tune;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@TeleOp(group = "tune")
public class DashboardMotorTuner extends OpMode {
    ArrayList<Pair<String, DcMotorEx>> motors = new ArrayList<>();

    @Override
    public void init() {
        FtcDashboard db = FtcDashboard.getInstance();

        for (DcMotorEx m : hardwareMap.getAll(DcMotorEx.class)) {
            String name = hardwareMap.getNamesOf(m).iterator().next();
            if (name == null) continue;
            this.motors.add(new Pair<>(name, m));

            db.addConfigVariable(this.getClass().getSimpleName(), name, new ValueProvider<String>() {
                final DcMotorEx servo = m;
                double power = 0.5;
                String verbatimInput = "";

                @Override
                public String get() {
                    return verbatimInput;
                }

                @Override
                public void set(String value) {
                    if (value.equals("")) {
                        this.servo.setMotorDisable();
                    } else {
                        try {
                            this.power = Double.parseDouble(value);
                        } catch (NumberFormatException e) {
                            return;
                        }

                        this.servo.setMotorEnable();
                        this.servo.setPower(this.power);
                    }

                    this.verbatimInput = value;
                }
            });
        }
    }

    @Override
    public void loop() {
        for (Pair<String, DcMotorEx> pair : this.motors) {
            telemetry.addData(pair.first, pair.second.getCurrentPosition());
        }
        telemetry.update();
    }
}
