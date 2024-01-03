package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class LED {
    enum Mode {
        RUNNING,
        IDLE
    }

    private Mode currentMode = Mode.IDLE;
    private Hardware hardware;

    public LED(Hardware hardware) {
        this.hardware = hardware;
    }

    public void update() {
        switch (currentMode) {
            case RUNNING:
                // TODO: do this
                break;
            case IDLE:
            default: // technically not needed but whatever
                idle();
                break;
        }
    }

    private void idle() {
        this.hardware.ledLeft.setPosition(mapValue(90));
        this.hardware.ledRight.setPosition(mapValue(84));
    }

    public void setMode(Mode newMode) {
        currentMode = newMode;
    }

    public Mode getMode() {
        return currentMode;
    }

    /**
     * Map a pattern number to a PWM value.
     * This equation was derived from the blinkin user manual and a whole lot of algebra.
     * The pattern numbers are available in the blinkin user manual.
     */
    private double mapValue(int n) {
        return (n + 49.5) / 200;
    }
}
