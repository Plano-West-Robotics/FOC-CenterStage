package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;

public class Intake {
    Hardware hardware;
    private double speed; // so that start and stop work properly
    private double oldSpeed;
    private boolean reversed;

    public Intake(OpMode opMode) {
        hardware = new Hardware(opMode);
    }

    public void update() {
        hardware.intake.setPower(speed * (reversed ? -1 : 1));
    }

    public void stop() {
        oldSpeed = speed;
        setSpeed(0);
    }

    public void start() {
        setSpeed(oldSpeed);
    }

    public void reverse() {
        reversed = !reversed;
    }

    public boolean isReversed() {
        return reversed;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double newSpeed) {
        speed = newSpeed;
    }
}
