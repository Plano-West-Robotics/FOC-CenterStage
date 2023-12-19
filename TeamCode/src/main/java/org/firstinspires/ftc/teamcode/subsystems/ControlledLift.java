package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class ControlledLift {
    private Hardware hardware;
    private int target;


    public ControlledLift(Hardware hardware) {
        this.hardware = hardware;
    }

    public void setTarget(double target) {
        this.target = (int)(target * (Lift.UPPER_LIMIT - Lift.LOWER_LIMIT) + Lift.LOWER_LIMIT);
    }

    public boolean update() {
        int liftLError = target - this.hardware.liftL.getCurrentPosition();
        int liftRError = target - this.hardware.liftR.getCurrentPosition();

        this.hardware.liftL.setPower(helper(liftLError) + Lift.GRAVITY_FEEDFORWARD);
        this.hardware.liftR.setPower(helper(liftRError) + Lift.GRAVITY_FEEDFORWARD);

        return Math.abs(liftLError) > 50 || Math.abs(liftRError) > 50;
    }

    public void stop() {
        if (this.target < 0.05) {
            this.hardware.liftL.setPower(0);
            this.hardware.liftR.setPower(0);
        } else {
            this.hardware.liftL.setPower(Lift.GRAVITY_FEEDFORWARD);
            this.hardware.liftR.setPower(Lift.GRAVITY_FEEDFORWARD);
        }
    }

    private static double helper(double error) {
        return 0.8 * Math.signum(error) * (Math.abs(error) > 300 ? 1: Math.sqrt(1 - Math.pow(1 - Math.abs(error) / 300, 2)));
    }
}
