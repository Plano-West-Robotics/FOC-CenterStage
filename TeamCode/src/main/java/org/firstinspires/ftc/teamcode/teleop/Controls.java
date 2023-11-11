package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.subsystems.Gamepads;

public class Controls {
    // ======================= GAMEPAD 1 ============================
    public static Gamepads.AnalogInput STRAIGHT = Gamepads.AnalogInput.GP1_LEFT_STICK_Y;
    public static Gamepads.AnalogInput STRAFE = Gamepads.AnalogInput.GP1_LEFT_STICK_X;
    public static Gamepads.AnalogInput TURN = Gamepads.AnalogInput.GP1_RIGHT_STICK_X;
    public static Gamepads.Button DRIVE_FASTER = Gamepads.Button.GP1_RIGHT_BUMPER;
    public static Gamepads.Button DRIVE_SLOWER = Gamepads.Button.GP1_LEFT_BUMPER;
    public static Gamepads.Button FIELD_ORIENTED = Gamepads.Button.GP1_SQUARE;
    public static Gamepads.Button RESET_IMU = Gamepads.Button.GP1_TRIANGLE;

    // ======================= GAMEPAD 2 ============================
    public static Gamepads.Button TOGGLE_INTAKE = Gamepads.Button.GP2_CROSS; // all of these are subject to change
    public static Gamepads.Button INTAKE_FASTER = Gamepads.Button.GP2_RIGHT_BUMPER;
    public static Gamepads.Button INTAKE_SLOWER = Gamepads.Button.GP2_LEFT_BUMPER;
    public static Gamepads.Button INTAKE_REVERSE = Gamepads.Button.GP2_CIRCLE;
    public static Gamepads.AnalogInput LIFT = Gamepads.AnalogInput.GP2_LEFT_STICK_Y;
    public static Gamepads.Button ARM_UP = Gamepads.Button.GP2_DPAD_UP;
    public static Gamepads.Button ARM_DOWN = Gamepads.Button.GP2_DPAD_DOWN;
    public static Gamepads.Button PEG_DISENGAGE = Gamepads.Button.GP2_TRIANGLE;
    public static Gamepads.Button PEG_ENGAGE = Gamepads.Button.GP2_SQUARE;
    public static Gamepads.Button LAUNCH_PLANE = Gamepads.Button.GP2_GUIDE;
}
