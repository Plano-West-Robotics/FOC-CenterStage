package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift {
    Hardware hardware;

    private static final int LIFT_ENCODER_LIMIT = 2000; // todo: tune these
    private static final int LIFT_DELTA = 50;

    public Lift(Hardware hardware) {
        this.hardware = hardware;
    }

    public void run(double speed) {
        this.hardware.liftL.setPower(speed);
        this.hardware.liftR.setPower(speed);
    }

    public void stop() {
        this.run(0);
    }
}
