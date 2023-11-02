package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;

public class Intake {
    Hardware hardware;
    private double speed;
    private boolean reversed;
    private boolean isRunning; // so that start and stop work properly

    public Intake(Hardware hardware, double speed) {
        this.hardware = hardware;
        this.speed = speed;
        this.reversed = false;

        // this is off by default because in most cases, the constructor will run in init
        // where we definitely don't want the motor to run
        // change this if you really need to but I doubt you will
        this.isRunning = false;
        hardware.intake.setPower(0);
        hardware.intake.setPower(0);
    }

    public void update() {
        double power = isRunning ? speed * (reversed ? -1 : 1) : 0;
        hardware.intake.setPower(power);
        // todo: make this manual if needed; might require making a separate subsystem
        hardware.ramp.setPower(power * 0.25);
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
