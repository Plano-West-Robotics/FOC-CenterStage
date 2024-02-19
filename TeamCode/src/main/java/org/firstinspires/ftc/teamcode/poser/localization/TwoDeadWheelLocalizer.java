package org.firstinspires.ftc.teamcode.poser.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.log.Plank;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Pose;

public class TwoDeadWheelLocalizer implements DeltaLocalizer {
    Hardware hardware;

    // sensor readings
    int backOdo;
    int rightOdo;
    public Angle imuYaw;

    long lastUpdate;

    public static final double MM_PER_ENCODER_TICK = ThreeDeadWheelLocalizer.MM_PER_ENCODER_TICK;

    public static final Distance BACK_ODO_LEVER_ARM = Distance.inMM(174.5);
    public static final Distance RIGHT_ODO_LEVER_ARM = Distance.inMM(162);
    public static int BACK_ODO_DIR = -1;
    public static int RIGHT_ODO_DIR = -1;

    public TwoDeadWheelLocalizer(Hardware hardware) {
        this.hardware = hardware;

        this.backOdo = hardware.backOdo.getCurrentPosition();
        this.rightOdo = hardware.rightOdo.getCurrentPosition();
        try { Thread.sleep(1000); }
        catch (InterruptedException ignored) { }
        this.imuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));

        lastUpdate = System.nanoTime();
    }

    public Pose updateWithDelta() {
        long now = System.nanoTime();
        double dt = (now - lastUpdate) / (1000 * 1000 * 1000.);
        lastUpdate = now;

        int newBackOdo = hardware.backOdo.getCurrentPosition();
        int newRightOdo = hardware.rightOdo.getCurrentPosition();
        Angle newImuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));

        double backOdoDiff = (newBackOdo - this.backOdo) * BACK_ODO_DIR * MM_PER_ENCODER_TICK;
        double rightOdoDiff = (newRightOdo - this.rightOdo) * RIGHT_ODO_DIR * MM_PER_ENCODER_TICK;
        Angle imuYawDiff = newImuYaw.sub(this.imuYaw);

        // yawDiff is in rad, rest in mm
        double yawDiff = imuYawDiff.modSigned().valInRadians();
        double relativeXDiff = rightOdoDiff - yawDiff * RIGHT_ODO_LEVER_ARM.valInMM();
        double relativeYDiff = yawDiff * BACK_ODO_LEVER_ARM.valInMM() - backOdoDiff;

        Plank log = hardware.log.chop("TwoDeadWheelLocalizer");
        log.addData("d/dt yaw", yawDiff / dt);
        log.addData("yaw", newImuYaw.valInDegrees());
        log.addData("d/dt rightOdo", rightOdoDiff / dt);
        log.addData("yawDiff * RIGHT_ODO_LEVER_ARM.valInMM()", yawDiff * RIGHT_ODO_LEVER_ARM.valInMM());
        log.addData("d/dt backOdo", backOdoDiff / dt);
        log.addData("yawDiff * BACK_ODO_LEVER_ARM.valInMM()", yawDiff * BACK_ODO_LEVER_ARM.valInMM());

        log.addData("relativeXDiff", relativeXDiff);
        log.addData("relativeYDiff", relativeYDiff);

        Pose delta = DeltaLocalizer.poseExpHelper(relativeXDiff, relativeYDiff, imuYawDiff);

        this.backOdo = newBackOdo;
        this.rightOdo = newRightOdo;
        this.imuYaw = newImuYaw;

        return delta;
    }
}
