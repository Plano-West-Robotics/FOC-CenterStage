package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;

public class Intake {
    Hardware hardware;
    private double speed; // so that start and stop work properly
    private boolean reversed;
    private boolean isRunning;

    public Intake(Hardware hardware, double speed) {
        this.hardware = hardware;
        this.speed = speed;
        this.reversed = false;
        // this is off by default because in most cases, the constructor will run in init
        // where we definitely don't want it to run
        // change this if you really need to but I doubt you will
        this.isRunning = false;
    }

    public void update() {
        hardware.intake.setPower(isRunning ? speed * (reversed ? -1 : 1) : 0);
    }

    public void stop() {
        isRunning = false;
    }

    public void start() {
        isRunning = true;
    }

    public void reverse() {
        reversed = !reversed;
    }

    public boolean isReversed() {
        return reversed;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void toggleRunning() {
        isRunning = !isRunning;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double newSpeed) {
        speed = newSpeed;
    }
}
