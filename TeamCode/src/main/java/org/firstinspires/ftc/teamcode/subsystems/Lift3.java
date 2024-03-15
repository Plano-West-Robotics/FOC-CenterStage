package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

public class Lift3 {
    private Hardware hardware;
    private ControlledArm arm;

    enum State {
        CONTINUUM,
        ARM_MOVING_DOWN,
        LIFT_MOVING_DOWN,
        LIFT_MOVING_UP,
        HANG_OVERRIDE,
    }

    private double power;
    private ControlledArm.ArmPosition armPosition;
    private State state;
    private boolean isGoingToTarget;
    private int target;
//    private int leftEncoder;
    private int rightEncoder;
    private int pos;

    public static final int LOWER_LIMIT = 500;
    public static final int UPPER_LIMIT = 2000;

    private static final int DELTA = 400;
    private static final double MAX_POWER = 0.9;

    public static final int HANG_HEIGHT = 1620;

    public static double GRAVITY_FEEDFORWARD = Lift.GRAVITY_FEEDFORWARD;

    public Lift3(Hardware hardware) {
        this.hardware = hardware;
        this.arm = new ControlledArm(hardware);

        this.power = 0;
        this.armPosition = ControlledArm.ArmPosition.UP;
        this.isGoingToTarget = false;
        this.state = State.LIFT_MOVING_DOWN;

//        leftEncoder = hardware.liftL.getCurrentPosition();
        rightEncoder = hardware.liftR.getCurrentPosition();
        pos = 0; // /shrug
    }

    public int getPos() {
        return this.pos;
    }

    public ControlledArm.ArmPosition getArmPosition() {
        return this.armPosition;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setArmPosition(ControlledArm.ArmPosition armPosition) {
        this.armPosition = armPosition;
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
        boolean atBottom = hardware.slideLimitSwitch.isPressed();
        if (atBottom) {
            pos = 10;
        }

        double outPower;
        if (override) {
            isGoingToTarget = false;
            if (state == State.HANG_OVERRIDE) {
                state = State.ARM_MOVING_DOWN;
            }
            outPower = power;
        } else if (state == State.HANG_OVERRIDE) {
            this.hardware.liftL.setPower(-0.97);
            this.hardware.liftR.setPower(-0.97);
            return;
        } else {
            if (isGoingToTarget) {
                int error = target - pos;
                outPower = helper2(error);
                if (Math.abs(error) < 25) {
                    isGoingToTarget = false;
                }
            } else {
                outPower = Range.clip(power, -MAX_POWER, MAX_POWER);
            }

            loop: while (true) switch (state) {
                case CONTINUUM:
                    arm.moveTo(armPosition);
                    arm.update();

                    if (pos < LOWER_LIMIT + 50 && outPower < 0) {
                        state = State.ARM_MOVING_DOWN;
                        continue;
                    }

//                    if (pos < LOWER_LIMIT - DELTA * MAX_POWER) {
//                        outPower = MAX_POWER;
//                    } else if (pos < LOWER_LIMIT + DELTA * MAX_POWER) {
//                        outPower = boundAtLeast(outPower, (double)(LOWER_LIMIT - pos) / DELTA);
//                    } else if (pos > UPPER_LIMIT + DELTA * MAX_POWER) {
//                        outPower = -MAX_POWER;
//                    } else if (pos > UPPER_LIMIT - DELTA * MAX_POWER) {
//                        outPower = boundAtMost(outPower, (double)(UPPER_LIMIT - pos) / DELTA);
//                    }
                    outPower = boundAtLeast(outPower, helper3(LOWER_LIMIT));
                    outPower = boundAtMost(outPower, helper3(UPPER_LIMIT));

                    break loop;
                case ARM_MOVING_DOWN:
                    arm.moveDown();
                    arm.update();

                    if (outPower > 0) {
                        state = State.CONTINUUM;
                        continue;
                    }

                    if (!arm.isBusy()) {
                        state = State.LIFT_MOVING_DOWN;
                        continue;
                    }

                    outPower = helper3(LOWER_LIMIT);

                    break loop;
                case LIFT_MOVING_DOWN:
                    if (outPower > 0) {
                        state = State.LIFT_MOVING_UP;
                        continue;
                    }

                    if (pos > 200) {
                        outPower = helper3(150);
                    } else {
                        outPower = -0.19;
                    }

                    break loop;
                case LIFT_MOVING_UP:
                    if (outPower < 0) {
                        state = State.LIFT_MOVING_DOWN;
                        continue;
                    }

                    if (pos > LOWER_LIMIT) {
                        state = State.CONTINUUM;
                        continue;
                    }

                    outPower = boundAtLeast(outPower, helper3(LOWER_LIMIT + 10));

                    break loop;
            }
        }
        this.hardware.liftL.setPower(outPower + GRAVITY_FEEDFORWARD);
        this.hardware.liftR.setPower(outPower + GRAVITY_FEEDFORWARD);
    }

    private static double boundAtLeast(double v, double bound) {
        return Math.max(v, bound);
    }

    private static double boundAtMost(double v, double bound) {
        return Math.min(v, bound);
    }

    private static double helper2(double error) {
        error /= 350;
        double tmp = Math.exp(3 * error);
        return MAX_POWER * (tmp - 1) / (tmp + 1);
    }

    private double helper3(double target) {
        return helper2(target - pos);
    }

    public void hang() {
        this.isGoingToTarget = false;
        this.state = State.HANG_OVERRIDE;
    }
}
