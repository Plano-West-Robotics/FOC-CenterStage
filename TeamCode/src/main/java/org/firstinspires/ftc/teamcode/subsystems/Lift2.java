package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift2 {
    Hardware hardware;

    private double power;
    private boolean isGoingToTarget;
    private int target;
//    private int leftEncoder;
    private int rightEncoder;
    private int pos;

    public static final int LOWER_LIMIT = 0;
    public static final int UPPER_LIMIT = 2000;

    // when within this amount of a limit, the slide cannot move toward that limit
    private static final int SMALL_DELTA = 50;
    // when within this amount of a limit, the slide cannot move faster than `MAX_NEAR_POWER` toward that limit
    private static final int DELTA = 150;
    // when outside the given bounds, the slide moves inward with at least this power
    private static final double OOB_POWER = 0.2;
    // when within `DELTA` of a limit, the slide can move no faster than this speed toward that limit
    private static final double MAX_NEAR_POWER = 0.4;

    public static final int NEAR_BOTTOM_RANGE = 200;
    public static final int HANG_HEIGHT = 1620;

    public static double GRAVITY_FEEDFORWARD = Lift.GRAVITY_FEEDFORWARD;

    public Lift2(Hardware hardware) {
        this.hardware = hardware;

        this.power = 0;
        this.isGoingToTarget = false;

//        leftEncoder = hardware.liftL.getCurrentPosition();
        rightEncoder = hardware.liftR.getCurrentPosition();
//        pos = (leftEncoder + rightEncoder) / 2; // /shrug
        pos = rightEncoder; // /shrug
    }

    public int getPos() {
        return this.pos;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setTarget(int target) {
        this.isGoingToTarget = true;
        this.target = target;
    }

    public void update(boolean override) {
//        int newLeftEncoder = this.hardware.liftL.getCurrentPosition();
        int newRightEncoder = this.hardware.liftR.getCurrentPosition();
//        int leftEncoderDiff = newLeftEncoder - leftEncoder;
        int rightEncoderDiff = newRightEncoder - rightEncoder;
//        leftEncoder = newLeftEncoder;
        rightEncoder = newRightEncoder;
//        pos += (leftEncoderDiff + rightEncoderDiff) / 2;
        pos += rightEncoderDiff;
        if (hardware.slideLimitSwitch.isPressed()) {
            pos = 10;
        }

        double outPower;
        if (override) {
            isGoingToTarget = false;
            outPower = power + GRAVITY_FEEDFORWARD;
        } else if (isGoingToTarget) {
            int error = target - pos;
            outPower = helper2(error) + GRAVITY_FEEDFORWARD;
            if (Math.abs(error) < 25) {
                isGoingToTarget = false;
            }
        } else {
            outPower = helper(pos, power);
        }
        this.hardware.liftL.setPower(outPower);
        this.hardware.liftR.setPower(outPower);
    }

    private static double helper(int encoder, double power) {
        if (encoder < LOWER_LIMIT) {
            power = boundAtLeast(power, OOB_POWER);
        // no SMALL_DELTA limit at bottom
//        } else if (encoder < LOWER_LIMIT + SMALL_DELTA) {
//            power = boundAtLeast(power, 0);
        } else if (encoder < LOWER_LIMIT + DELTA) {
            power = boundAtLeast(power, -MAX_NEAR_POWER);
        } else if (encoder > UPPER_LIMIT) {
            power = boundAtMost(power, -OOB_POWER);
        } else if (encoder > UPPER_LIMIT - SMALL_DELTA) {
            power = boundAtMost(power, 0);
        } else if (encoder > UPPER_LIMIT - DELTA) {
            power = boundAtMost(power, MAX_NEAR_POWER);
        }

        power += GRAVITY_FEEDFORWARD;

        return power;
    }

    private static double boundAtLeast(double v, double bound) {
        return Math.max(v, bound);
    }

    private static double boundAtMost(double v, double bound) {
        return Math.min(v, bound);
    }

    private static double helper2(double error) {
        error /= 300;
        double tmp = Math.exp(3 * error);
        return 0.8 * (tmp - 1) / (tmp + 1);

//        return 0.8 * Math.signum(error) * (Math.abs(error) > 300 ? 1 : Math.sqrt(1 - Math.pow(1 - Math.abs(error) / 300, 2)));
    }

    public void stop() {
        this.hardware.liftL.setPower(0);
        this.hardware.liftR.setPower(0);
    }
}
