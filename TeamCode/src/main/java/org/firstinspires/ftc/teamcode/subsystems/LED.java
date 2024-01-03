package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class LED {
    public enum Mode {
        RUNNING,
        IDLE
    }

    private Mode currentMode = Mode.IDLE;
    private final Hardware hardware;
    public final Sensor sensor;

    /**
     * color to pattern number mappings
     * making mr london proud fr
     * "no magic numbers!" - nerd emoji
     */
    enum ColorPattern {
        FOC_GREEN(90), FOC_YELLOW(84),

        GREEN(89), PURPLE(96), YELLOW(85), WHITE(97),

        NONE(100);

        public final int val;

        ColorPattern(int i) {
            val = i;
        }
    }

    public LED(Hardware hardware) {
        this.hardware = hardware;
        sensor = new Sensor(this.hardware);
    }

    public void update() {
        switch (currentMode) {
            case RUNNING:
                showColors();
                break;
            case IDLE:
            default: // technically not needed but whatever
                idle();
                break;
        }
    }

    private void showColors() {
        sensor.update();
        // it's bigger on the inside
        this.hardware.ledLeft.setPwmEnable();
        this.hardware.ledRight.setPwmEnable();

        this.hardware.ledLeft.setPosition(mapValue(sensor.top_state.toColorPattern()));
        this.hardware.ledRight.setPosition(mapValue(sensor.bottom_state.toColorPattern()));
    }

    private void idle() {
        this.hardware.ledLeft.setPwmDisable(); // if PWM is off, the blinkin falls back to the idle pattern
        this.hardware.ledRight.setPwmDisable();
    }

    public void setMode(Mode newMode) {
        currentMode = newMode;
    }

    public Mode getMode() {
        return currentMode;
    }

    /**
     * Map a pattern number to a PWM value.
     */
    private double mapValue(ColorPattern pattern) {
        return (pattern.val - 1) / 99.0;
    }
}
