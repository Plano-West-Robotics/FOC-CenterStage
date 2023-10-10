package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

public class Controls {
    public static GamepadWrapper.AnalogInput GP1_STRAIGHT = GamepadWrapper.AnalogInput.LEFT_STICK_Y;
    public static GamepadWrapper.AnalogInput GP1_STRAFE = GamepadWrapper.AnalogInput.LEFT_STICK_X;
    public static GamepadWrapper.AnalogInput GP1_TURN = GamepadWrapper.AnalogInput.RIGHT_STICK_X;
    public static GamepadWrapper.Button GP1_FASTER = GamepadWrapper.Button.RIGHT_BUMPER;
    public static GamepadWrapper.Button GP1_SLOWER = GamepadWrapper.Button.LEFT_BUMPER;
    public static GamepadWrapper.Button GP1_FIELD_ORIENTED = GamepadWrapper.Button.X;
    public static GamepadWrapper.Button GP1_RESET_IMU = GamepadWrapper.Button.Y;
}
