package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift {
    Hardware hardware;

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
