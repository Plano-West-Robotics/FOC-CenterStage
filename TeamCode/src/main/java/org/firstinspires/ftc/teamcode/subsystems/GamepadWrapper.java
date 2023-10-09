package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {
    private Gamepad prevState;
    private Gamepad currState;

    public enum Button {
        A,
        B,
        X,
        Y,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
    }

    public enum AnalogInput {
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
        LEFT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_X,
        RIGHT_STICK_Y,
    }

    public GamepadWrapper(Gamepad gp) {
        prevState = new Gamepad();
        currState = gp;
    }

    public GamepadWrapper() {
        this(new Gamepad());
    }

    public boolean isPressed(Button button) {
        return isPressed(button, currState);
    }

    private boolean isPressed(Button button, Gamepad gamepad) {
        switch (button) {
            case A: return gamepad.a;
            case B: return gamepad.b;
            case X: return gamepad.x;
            case Y: return gamepad.y;
            case LEFT_BUMPER: return gamepad.left_bumper;
            case RIGHT_BUMPER: return gamepad.right_bumper;
            case LEFT_STICK_BUTTON: return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON: return gamepad.right_stick_button;
            case DPAD_UP: return gamepad.dpad_up;
            case DPAD_DOWN: return gamepad.dpad_down;
            case DPAD_LEFT: return gamepad.dpad_left;
            case DPAD_RIGHT: return gamepad.dpad_right;

            default: throw new RuntimeException("unreachable code entered");
        }
    }

    public double getAnalogValue(AnalogInput button) {
        return getAnalogValue(button, currState);
    }

    private double getAnalogValue(AnalogInput button, Gamepad gamepad) {
        switch (button) {
            case LEFT_TRIGGER: return gamepad.left_trigger;
            case RIGHT_TRIGGER: return gamepad.right_trigger;
            case LEFT_STICK_X: return gamepad.left_stick_x;
            case LEFT_STICK_Y: return -gamepad.left_stick_y;
            case RIGHT_STICK_X: return gamepad.right_stick_x;
            case RIGHT_STICK_Y: return -gamepad.right_stick_y;

            default: throw new RuntimeException("unreachable code entered");
        }
    }

    public boolean justPressed(Button button) {
        return (!isPressed(button, prevState) && isPressed(button, currState));
    }

    public boolean justReleased(Button button) {
        return (isPressed(button, prevState) && !isPressed(button, currState));
    }

    public void update(Gamepad newState) {
        prevState.copy(currState);
        currState.copy(newState);
    }
}