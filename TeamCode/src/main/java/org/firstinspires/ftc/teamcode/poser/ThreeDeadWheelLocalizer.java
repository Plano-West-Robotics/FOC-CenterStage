package org.firstinspires.ftc.teamcode.poser;

import org.firstinspires.ftc.teamcode.Hardware;

public class ThreeDeadWheelLocalizer implements Localizer {
    Hardware hardware;

    private Pose poseEstimate;

    // sensor readings
    int leftOdo;
    int backOdo;
    int rightOdo;

    public static final double MM_PER_ENCODER_TICK = (35 * Math.PI) / 8192;

    public static final Distance LEFT_ODO_LEVER_ARM = null; // TODO
    public static final Distance BACK_ODO_LEVER_ARM = Distance.inMM(180);
    public static final Distance RIGHT_ODO_LEVER_ARM = Distance.inMM(165);
    public static int LEFT_ODO_DIR = 0; // TODO
    public static int BACK_ODO_DIR = -1; // 1 for CCW positive, -1 for CW
    public static int RIGHT_ODO_DIR = -1;

    public ThreeDeadWheelLocalizer(Hardware hardware, Pose initialPose) {
        this.hardware = hardware;
        this.poseEstimate = initialPose;

        this.leftOdo = hardware.leftOdo.getCurrentPosition();
        this.backOdo = hardware.backOdo.getCurrentPosition();
        this.rightOdo = hardware.rightOdo.getCurrentPosition();
    }

    public void update() {
        int newLeftOdo = hardware.leftOdo.getCurrentPosition();
        int newBackOdo = hardware.backOdo.getCurrentPosition();
        int newRightOdo = hardware.rightOdo.getCurrentPosition();

        double leftOdoDiff = (newLeftOdo - this.leftOdo) * LEFT_ODO_DIR * MM_PER_ENCODER_TICK;
        double backOdoDiff = (newBackOdo - this.backOdo) * BACK_ODO_DIR * MM_PER_ENCODER_TICK;
        double rightOdoDiff = (newRightOdo - this.rightOdo) * RIGHT_ODO_DIR * MM_PER_ENCODER_TICK;

        // yawDiff is in rad, rest in mm
        double yawDiff = (leftOdoDiff + rightOdoDiff) / (LEFT_ODO_LEVER_ARM.valInMM() + RIGHT_ODO_LEVER_ARM.valInMM());
        double relativeXDiff = (-leftOdoDiff + rightOdoDiff) / 2.;
        double relativeYDiff = yawDiff * BACK_ODO_LEVER_ARM.valInMM() - backOdoDiff;

        this.poseEstimate = this.poseEstimate.then(Localizer.poseExpHelper(relativeXDiff, relativeYDiff, yawDiff));

        this.leftOdo = newLeftOdo;
        this.backOdo = newBackOdo;
        this.rightOdo = newRightOdo;
    }

    public Pose getPoseEstimate() {
        return this.poseEstimate;
    }
}
