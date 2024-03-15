package org.firstinspires.ftc.teamcode.poser.localization;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Pose;

public class ThreeDeadWheelLocalizer implements DeltaLocalizer {
    Hardware hardware;

    // sensor readings
    int leftOdo;
    int backOdo;
    int rightOdo;

    public static final double MM_PER_ENCODER_TICK = (35 * Math.PI) / 8192;
    public static final double MM_PER_ENCODER_TICK_2 = (48 * Math.PI) / 2000;

    public static final Distance LEFT_ODO_LEVER_ARM = Distance.inMM(161);
    public static final Distance BACK_ODO_LEVER_ARM = Distance.inMM(174.5);
    public static final Distance RIGHT_ODO_LEVER_ARM = Distance.inMM(162);
    public static int LEFT_ODO_DIR = 1; // 1 for CCW positive, -1 for CW
    public static int BACK_ODO_DIR = -1;
    public static int RIGHT_ODO_DIR = -1;
// as
    public ThreeDeadWheelLocalizer(Hardware hardware) {
        this.hardware = hardware;

        this.leftOdo = hardware.leftOdo.getCurrentPosition();
        this.backOdo = hardware.backOdo.getCurrentPosition();
        this.rightOdo = hardware.rightOdo.getCurrentPosition();
    }

    public Pose updateWithDelta() {
        int newLeftOdo = hardware.leftOdo.getCurrentPosition();
        int newBackOdo = hardware.backOdo.getCurrentPosition();
        int newRightOdo = hardware.rightOdo.getCurrentPosition();

        double leftOdoDiff = (newLeftOdo - this.leftOdo) * LEFT_ODO_DIR * MM_PER_ENCODER_TICK_2; // this one is gobilda
        double backOdoDiff = (newBackOdo - this.backOdo) * BACK_ODO_DIR * MM_PER_ENCODER_TICK;
        double rightOdoDiff = (newRightOdo - this.rightOdo) * RIGHT_ODO_DIR * MM_PER_ENCODER_TICK;

        // yawDiff is in rad, rest in mm
        double yawDiff = (leftOdoDiff + rightOdoDiff) / (LEFT_ODO_LEVER_ARM.valInMM() + RIGHT_ODO_LEVER_ARM.valInMM());
        double relativeXDiff = (-leftOdoDiff + rightOdoDiff) / 2.;
        double relativeYDiff = yawDiff * BACK_ODO_LEVER_ARM.valInMM() - backOdoDiff;

        Pose delta = DeltaLocalizer.poseExpHelper(relativeXDiff, relativeYDiff, yawDiff);

        this.leftOdo = newLeftOdo;
        this.backOdo = newBackOdo;
        this.rightOdo = newRightOdo;

        return delta;
    }
}
