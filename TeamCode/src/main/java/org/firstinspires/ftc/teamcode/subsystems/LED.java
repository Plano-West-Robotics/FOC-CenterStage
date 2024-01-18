package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Hardware;

public class LED {
    public enum Mode {
        RUNNING,
        IDLE,
        EJECT_OVERRIDE
    }

    private Mode currentMode = Mode.IDLE;
    private final Hardware hardware;
    public final Sensor sensor;

    public LED(Hardware hardware) {
        this.hardware = hardware;
        sensor = new Sensor(this.hardware);
    }

    public void update() {
        switch (currentMode) {
            case RUNNING:
                showColors();
                break;
            case EJECT_OVERRIDE:
                ejectOverride();
                break;
            case IDLE:
            default: // technically not needed but whatever
                idle();
                break;
        }
    }

    private void showColors() {
        sensor.update();

        this.hardware.ledLeft.setPattern(sensor.top_state.toColorPattern());
        this.hardware.ledRight.setPattern(sensor.bottom_state.toColorPattern());
    }

    private void idle() {
        this.hardware.ledLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        this.hardware.ledRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    private void ejectOverride() {
        this.hardware.ledLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        this.hardware.ledRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setMode(Mode newMode) {
        currentMode = newMode;
    }

    public Mode getMode() {
        return currentMode;
    }
}
