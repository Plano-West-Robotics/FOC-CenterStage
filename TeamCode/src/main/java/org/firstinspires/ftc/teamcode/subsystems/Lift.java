package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift {
    Hardware hardware;

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

    public Lift(Hardware hardware) {
        this.hardware = hardware;
    }

    public void run(double power) {
        int leftEncoder = this.hardware.liftL.getCurrentPosition();
        int rightEncoder = this.hardware.liftR.getCurrentPosition();

        double leftPower = helper(leftEncoder, power);
        double rightPower = helper(rightEncoder, power);

        this.hardware.liftL.setPower(leftPower);
        this.hardware.liftR.setPower(rightPower);
    }

    private static double helper(int encoder, double power) {
//        if (encoder < LOWER_LIMIT) {
//            power = boundAtLeast(power, OOB_POWER);
//        } else if (encoder < LOWER_LIMIT + SMALL_DELTA) {
//            power = boundAtLeast(power, 0);
//        } else if (encoder < LOWER_LIMIT + DELTA) {
//            power = boundAtLeast(power, -MAX_NEAR_POWER);
//        } else if (encoder > UPPER_LIMIT) {
//            power = boundAtMost(power, -OOB_POWER);
//        } else if (encoder > UPPER_LIMIT - SMALL_DELTA) {
//            power = boundAtMost(power, 0);
//        } else if (encoder > UPPER_LIMIT - DELTA) {
//            power = boundAtMost(power, MAX_NEAR_POWER);
//        }

        return power * 0.75;
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
