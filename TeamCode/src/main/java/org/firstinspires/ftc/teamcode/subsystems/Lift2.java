package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift2 {
    Hardware hardware;

    private double power;
    private int leftEncoder;
    private int rightEncoder;
    private int pos;

    public static final int LOWER_LIMIT = 0;
    public static final int UPPER_LIMIT = 2000;

    // when within this amount of a limit, the slide cannot move toward that limit
    private static final int SMALL_DELTA = 25;
    // when within this amount of a limit, the slide cannot move faster than `MAX_NEAR_POWER` toward that limit
    private static final int DELTA = 100;
    // when outside the given bounds, the slide moves inward with at least this power
    private static final double OOB_POWER = 0.4;
    // when within `DELTA` of a limit, the slide can move no faster than this speed toward that limit
    private static final double MAX_NEAR_POWER = 0.6;

    public static final int NEAR_BOTTOM_RANGE = 200;

    public static double GRAVITY_FEEDFORWARD = 0.05;

    public Lift2(Hardware hardware) {
        this.hardware = hardware;

        this.power = 0;

        leftEncoder = hardware.liftL.getCurrentPosition();
        rightEncoder = hardware.liftR.getCurrentPosition();
        pos = (leftEncoder + rightEncoder) / 2; // /shrug
    }

    public int getPos() {
        return this.pos;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void update() {
        int newLeftEncoder = this.hardware.liftL.getCurrentPosition();
        int newRightEncoder = this.hardware.liftR.getCurrentPosition();
        int leftEncoderDiff = newLeftEncoder - leftEncoder;
        int rightEncoderDiff = newRightEncoder - rightEncoder;
        leftEncoder = newLeftEncoder;
        rightEncoder = newRightEncoder;
        pos += (leftEncoderDiff + rightEncoderDiff) / 2;
        if (hardware.slideLimitSwitch.isPressed()) {
            pos = 10;
        }

        double adjustedPower = helper(pos, power);
        this.hardware.liftL.setPower(adjustedPower);
        this.hardware.liftR.setPower(adjustedPower);
    }

    private static double helper(int encoder, double power) {
        power += GRAVITY_FEEDFORWARD;

        if (encoder < LOWER_LIMIT) {
            power = boundAtLeast(power, OOB_POWER);
        } else if (encoder < LOWER_LIMIT + SMALL_DELTA) {
            power = boundAtLeast(power, 0);
        } else if (encoder < LOWER_LIMIT + DELTA) {
            power = boundAtLeast(power, -MAX_NEAR_POWER);
        } else if (encoder > UPPER_LIMIT) {
            power = boundAtMost(power, -OOB_POWER);
        } else if (encoder > UPPER_LIMIT - SMALL_DELTA) {
            power = boundAtMost(power, 0);
        } else if (encoder > UPPER_LIMIT - DELTA) {
            power = boundAtMost(power, MAX_NEAR_POWER);
        }

        return power;
    }

    private static double boundAtLeast(double v, double bound) {
        return Math.max(v, bound);
    }

    private static double boundAtMost(double v, double bound) {
        return Math.min(v, bound);
    }

    public void stop() {
        this.hardware.liftL.setPower(0);
        this.hardware.liftR.setPower(0);
    }
}
