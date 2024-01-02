package org.firstinspires.ftc.teamcode.poser;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;

public class TwoDeadWheelLocalizer implements Localizer {
    Hardware hardware;

    private Pose poseEstimate;

    // sensor readings
    int leftOdo;
    int backOdo;
    Angle imuYaw;

    public static final double MM_PER_ENCODER_TICK = ThreeDeadWheelLocalizer.MM_PER_ENCODER_TICK;

    public static final Distance LEFT_ODO_LEVER_ARM = Distance.inMM(165);
    public static final Distance BACK_ODO_LEVER_ARM = Distance.inMM(180);
    public static int LEFT_ODO_DIR = -1;
    public static int BACK_ODO_DIR = 1;

    public TwoDeadWheelLocalizer(Hardware hardware, Pose initialPose) {
        this.hardware = hardware;
        this.poseEstimate = initialPose;

        this.leftOdo = hardware.leftOdo.getCurrentPosition();
        this.backOdo = hardware.backOdo.getCurrentPosition();
        try { Thread.sleep(1000); }
        catch (InterruptedException ignored) { }
        this.imuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));
    }

    public void update() {
        int newLeftOdo = hardware.leftOdo.getCurrentPosition();
        int newBackOdo = hardware.backOdo.getCurrentPosition();
        Angle newImuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));

        double leftOdoDiff = (newLeftOdo - this.leftOdo) * LEFT_ODO_DIR * MM_PER_ENCODER_TICK;
        double backOdoDiff = (newBackOdo - this.backOdo) * BACK_ODO_DIR * MM_PER_ENCODER_TICK;
        Angle imuYawDiff = newImuYaw.sub(this.imuYaw);

        // yawDiff is in rad, rest in mm
        double yawDiff = imuYawDiff.valInRadians();
        double relativeXDiff = yawDiff * LEFT_ODO_LEVER_ARM.valInMM() - leftOdoDiff;
        double relativeYDiff = yawDiff * BACK_ODO_LEVER_ARM.valInMM() - backOdoDiff;

        this.poseEstimate = this.poseEstimate.then(Localizer.poseExpHelper(relativeXDiff, relativeYDiff, imuYawDiff));

        this.leftOdo = newLeftOdo;
        this.backOdo = newBackOdo;
        this.imuYaw = newImuYaw;
    }

    public Pose getPoseEstimate() {
        return this.poseEstimate;
    }
}
