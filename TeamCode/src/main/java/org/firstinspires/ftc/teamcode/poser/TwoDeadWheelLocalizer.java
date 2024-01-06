package org.firstinspires.ftc.teamcode.poser;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;

public class TwoDeadWheelLocalizer implements Localizer {
    Hardware hardware;

    private Pose poseEstimate;

    // sensor readings
    int backOdo;
    int rightOdo;
    Angle imuYaw;

    public static final double MM_PER_ENCODER_TICK = ThreeDeadWheelLocalizer.MM_PER_ENCODER_TICK;

    public static final Distance BACK_ODO_LEVER_ARM = Distance.inMM(175);
    public static final Distance RIGHT_ODO_LEVER_ARM = Distance.inMM(165);
    public static int BACK_ODO_DIR = -1;
    public static int RIGHT_ODO_DIR = -1;

    public TwoDeadWheelLocalizer(Hardware hardware, Pose initialPose) {
        this.hardware = hardware;
        this.poseEstimate = initialPose;

        this.backOdo = hardware.backOdo.getCurrentPosition();
        this.rightOdo = hardware.rightOdo.getCurrentPosition();
        try { Thread.sleep(1000); }
        catch (InterruptedException ignored) { }
        this.imuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));
    }

    public void update() {
        int newBackOdo = hardware.backOdo.getCurrentPosition();
        int newRightOdo = hardware.rightOdo.getCurrentPosition();
        Angle newImuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));

        double backOdoDiff = (newBackOdo - this.backOdo) * BACK_ODO_DIR * MM_PER_ENCODER_TICK;
        double rightOdoDiff = (newRightOdo - this.rightOdo) * RIGHT_ODO_DIR * MM_PER_ENCODER_TICK;
        Angle imuYawDiff = newImuYaw.sub(this.imuYaw);

        // yawDiff is in rad, rest in mm
        double yawDiff = imuYawDiff.valInRadians();
        double relativeXDiff = rightOdoDiff - yawDiff * RIGHT_ODO_LEVER_ARM.valInMM();
        double relativeYDiff = yawDiff * BACK_ODO_LEVER_ARM.valInMM() - backOdoDiff;

        this.poseEstimate = this.poseEstimate.then(Localizer.poseExpHelper(relativeXDiff, relativeYDiff, imuYawDiff));

        this.backOdo = newBackOdo;
        this.rightOdo = newRightOdo;
        this.imuYaw = newImuYaw;
    }

    public Pose getPoseEstimate() {
        return this.poseEstimate;
    }
}
