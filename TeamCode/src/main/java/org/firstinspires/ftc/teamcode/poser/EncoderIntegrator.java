package org.firstinspires.ftc.teamcode.poser;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;

public class EncoderIntegrator {
    Hardware hardware;

    public Pose poseEstimate;

    // sensor readings
    int fl;
    int fr;
    int bl;
    int br;
    Angle imuYaw;

    public static final double MM_PER_ENCODER_TICK = (96 * Math.PI) / 537.7;
    public static final double TRACK_WIDTH = 355;
    public static final double DEGREES_PER_MM = 360 / (TRACK_WIDTH * Math.PI);

    public static final double X_AXIS_CALIB = 1.0390625;
    public static final double Y_AXIS_CALIB = 1.2734375;

    public EncoderIntegrator(Hardware hardware, Pose initialPose) {
        this.hardware = hardware;
        this.poseEstimate = initialPose;

        this.fl = hardware.fl.getCurrentPosition();
        this.fr = hardware.fr.getCurrentPosition();
        this.bl = hardware.bl.getCurrentPosition();
        this.br = hardware.br.getCurrentPosition();
        try { Thread.sleep(1000); }
        catch (InterruptedException ignored) { }
        this.imuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));
    }

    private static double sinc(double x) {
        return x == 0 ? 1 : (Math.sin(x) / x);
    }

    // this function has no generally well-known name, but its similar to `sinc` so im calling it `cosc`
    private static double cosc(double x) {
        return x == 0 ? 0 : ((1 - Math.cos(x)) / x);
    }

    public void update() {
        int newFl = hardware.fl.getCurrentPosition();
        int newFr = hardware.fr.getCurrentPosition();
        int newBl = hardware.bl.getCurrentPosition();
        int newBr = hardware.br.getCurrentPosition();
        Angle newImuYaw = Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS));

        double flDiff = newFl - this.fl;
        double frDiff = newFr - this.fr;
        double blDiff = newBl - this.bl;
        double brDiff = newBr - this.br;
        Angle imuYawDiff = newImuYaw.sub(this.imuYaw);

        // fl = powerY + powerX + turn + noop
        // fr = powerY - powerX - turn + noop
        // bl = powerY - powerX + turn - noop
        // br = powerY + powerX - turn - noop

        double relativeXDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.) * MM_PER_ENCODER_TICK / X_AXIS_CALIB;
        double relativeYDiff = ((-flDiff + frDiff + blDiff - brDiff) / 4.) * MM_PER_ENCODER_TICK / Y_AXIS_CALIB;
        // unused
        // double turnDiff = ((flDiff - frDiff + blDiff - brDiff) / 4.) * MM_PER_ENCODER_TICK * DEGREES_PER_MM;
        // double noopDiff = (flDiff + frDiff - blDiff - brDiff) / 4.;

        double poseExponentiationX = cosc(imuYawDiff.valInRadians());
        double poseExponentiationY = sinc(imuYawDiff.valInRadians());

        Distance2 relativePosDiff = Distance2.inMM(
                relativeXDiff * poseExponentiationY - relativeYDiff * poseExponentiationX,
                relativeXDiff * poseExponentiationX + relativeYDiff * poseExponentiationY
        );

        Pose poseDiff = new Pose(
                relativePosDiff,
                imuYawDiff
        );

        this.poseEstimate = this.poseEstimate.then(poseDiff);

        this.fl = newFl;
        this.fr = newFr;
        this.bl = newBl;
        this.br = newBr;
        this.imuYaw = newImuYaw;
    }
}
