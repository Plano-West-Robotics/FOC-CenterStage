package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift {
    Hardware hardware;

    public static final int LOWER_LIMIT = 0;
    public static final int UPPER_LIMIT = 2000;

    public static double GRAVITY_FEEDFORWARD = 0.05;

    public Lift(Hardware hardware) {
        this.hardware = hardware;
    }

    public void run(double power) {
        this.hardware.liftL.setPower(power + GRAVITY_FEEDFORWARD);
        this.hardware.liftR.setPower(power + GRAVITY_FEEDFORWARD);
    }

    public void stop() {
        this.hardware.liftL.setPower(0);
        this.hardware.liftR.setPower(0);
    }
}
