package org.firstinspires.ftc.teamcode.inchworm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * InchWorm2 is a movement system aimed at making autonomous coding easy and effective.
 */
public class InchWormBackup {
    /**
     * Encoder ticks per motor revolution for your drive motors. You can find this information online.
     */
    public static final double TICKS_PER_REV = 537.7;
    /**
     * Diameter of your mecanum wheels in inches.
     */
    public static final double WHEEL_DIAMETER_INCHES = 9.6/2.54;
    /**
     * Encoder ticks per inch rotated
     */
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    public final IMU imu;
    public final PositionTracker tracker = new PositionTracker();
    private int loopsCorrect = 0;
    /**
     * Maximum translational velocity in encoder ticks/second. Find this using the SpeedTuner
     */
    private static final double MAX_VEL = 1721.398;
    /**
     * Maximum angular velocity in degrees/second. Find this using the SpeedTuner.
     */
    private static final double MAX_ANG_VEL = -149.353;
    /**
     * PID controllers. <b>coefficients for controllerX and controllerY should be THE SAME!</b>
     * Tune controllerX and controllerY with the TranslationalPIDTuner, and tune controllerTheta with the TurnPIDTuner.
     */
    private final PIDController controllerX = new PIDController(1.75, 0, 0, 0);
    private final PIDController controllerY = new PIDController(1.75, 0, 0, 0);
    private final PIDController controllerTheta = new PIDController(2.5, 0, 0, 0);

    private final LinearOpMode opMode;

    /**
     * speed multiplier, multiplied by the PID outputs.
     */
    private double speed = 1;

    /**
     * Whether to draw a representation of the robot to FTC Dashboard or not.
     */
    private static final boolean DASHBOARD_DRAW = true;

    public InchWormBackup(LinearOpMode mode) {
        opMode = mode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                // TODO: change these parameters if they are not accurate
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this is a temporary measure. modes will be reset once actually moving
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // counteract inertia
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Move to a specific position on the field.
     * @param pose Position to move to. For now, must be in inches and radians.
     */
    public void moveTo(Pose pose) {
        // convert pose in inches to pose in ticks & normalize angle to [-π, π] radians
        pose = pose.toTicks().normalizeAngle();
        controllerX.setTarget(pose.x);
        controllerY.setTarget(pose.y);
        controllerTheta.setTarget(Math.toDegrees(pose.theta));
        controllerX.reset();
        controllerY.reset();
        controllerTheta.reset();
        Pose current = tracker.currentPos.normalizeAngle();

        while (isBusy(pose, current)) {
            current = tracker.currentPos.normalizeAngle();
            opMode.telemetry.addLine(current.toDegrees().toString());
            double angError = Math.toDegrees(angleDiff(pose.theta, current.theta));
            opMode.telemetry.addData("angError", angError);
            Pose out = new Pose(controllerX.calculate(current.x), controllerY.calculate(current.y), controllerTheta.calculateWithError(angError));

            out = out.rot(-current.theta);
            out = new Pose(out.x / MAX_VEL, out.y / MAX_VEL, out.theta / MAX_ANG_VEL);
            opMode.telemetry.addLine(out.toString());
            opMode.telemetry.update();

            double voltageCompensation = 12 / getBatteryVoltage();
            moveWheels(out.x, out.y, out.theta, getSpeedMultiplier() * voltageCompensation);
            tracker.update();
        }

        stop();
    }

    /**
     * Move to a certain (x, y) on the field.
     * @param x x coordinate to move to. for now, must be in inches
     * @param y y coordinate to move to. for now, must be in inches
     */
    public void moveTo(double x, double y) {
        moveTo(new Pose(x, y, tracker.currentPos.theta));
    }

    /**
     * Move to a certain (x, y, θ) on the field.
     * @param x x coordinate to move to. for now, must be in inches.
     * @param y y coordinate to move to. for now, must be in inches.
     * @param theta angle to turn to. for now, must be in radians.
     */
    public void moveTo(double x, double y, double theta) {
        moveTo(new Pose(x, y, theta));
    }

    private void setModes(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    /**
     * Whether motors are currently attempting to run to the target position.
     * Also includes a safeguard for stopping the opMode mid-move.
     * @return Whether all motors are busy, AND if the opMode is still running.
     */
    public boolean isBusy(Pose target, Pose current) {
        if (
                Math.abs(target.x - current.x) <= 20 &&
                        Math.abs(target.y - current.y) <= 20 &&
                        Math.abs(Math.toDegrees(angleDiff(target.theta, current.theta))) <= 5
        ) {
            loopsCorrect++;
        } else loopsCorrect = 0;

        return loopsCorrect <= 35 && opMode.opModeIsActive();
    }

    /**
     * Stops all motors
     */
    private void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void moveWheels(double powerX, double powerY, double turn, double speed) {
        double flPower = (powerX - powerY + turn) * speed;
        double frPower = (powerX + powerY - turn) * speed;
        double blPower = (powerX + powerY + turn) * speed;
        double brPower = (powerX - powerY - turn) * speed;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl,fr,bl,br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

    }

    /**
     * normalizes theta into [0, 2π)
     * @param theta angle to normalize in radians
     * @return theta normalized into [0, 2π)
     */
    private static double modAngle(double theta) {
        return theta % (2 * Math.PI);
    }

    /**
     * Returns the real (smallest) difference between two angles.
     * @param a first angle (in radians)
     * @param b second angle (in radians)
     * @return smallest difference between the two angles, within range [-π, π)
     */
    private static double angleDiff(double a, double b) {
        return modAngle(a - b + Math.PI) - Math.PI;
    }

    /**
     * Get yaw from the IMU.
     * @param angleUnit Unit of the result.
     * @return yaw angle from the IMU, in `angleUnit` units.
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Get yaw from the IMU in radians.
     * @return yaw angle from the IMU in radians.
     */
    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }

    private double getBatteryVoltage() {
        // returns the root of mean of the squares of all the battery voltages
        double totalSquares = 0;
        int numSeen = 0;
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            totalSquares += Math.pow(sensor.getVoltage(), 2);
            numSeen++;
        }

        return Math.sqrt(totalSquares / numSeen);
    }

    /**
     * Set the speed multiplier of the robot.
     * @param x new speed to set
     */
    public void setSpeedMultiplier(double x) {
        speed = x;
    }

    /**
     * Get the current speed multiplier.
     * @return current speed multiplier
     */
    public double getSpeedMultiplier() {
        return speed;
    }

    private void drawPose(Pose currentPose) {
        if (!DASHBOARD_DRAW) return;

        TelemetryPacket packet = new TelemetryPacket();
        double x = currentPose.x / TPI * (24 / 23.625);
        double y = currentPose.y / TPI * (24 / 23.625);
        double cos = Math.cos(currentPose.theta);
        double sin = Math.sin(currentPose.theta);
        double r = 9; // customize this if needed

        packet.fieldOverlay()
                .strokeCircle(x, y, r)
                .strokeLine(x, y, r * cos + x, r * sin + y);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static class Pose {
        double x;
        double y;
        double theta = 0;

        public Pose(double X, double Y) {
            x = X;
            y = Y;
        }

        public Pose(double X, double Y, double angle) {
            x = X;
            y = Y;
            theta = angle;
        }

        /**
         * Convert a pose where x and y are in inches to a pose where x and y in ticks.
         * @return a pose where x and y in ticks.
         */
        public Pose toTicks() {
            return new Pose(this.x * TPI, this.y * TPI, this.theta);
        }

        /**
         * Normalizes theta into [0, 2π).
         * @see super.modAngle(double)
         * @return A new pose with theta normalized into [0, 2π).
         */
        public Pose normalizeAngle() {
            return new Pose(this.x, this.y, modAngle(this.theta));
        }

        /**
         * Rotates this pose by an angle.
         * @param angle angle to rotate by
         * @return new pose that is rotated by the angle
         */
        public Pose rot(double angle) {
            double rotX = this.x * Math.cos(angle) - this.y * Math.sin(angle);
            double rotY = this.x * Math.sin(angle) + this.y * Math.cos(angle);

            return new Pose(rotX, rotY, this.theta);
        }

        public Pose add(Pose other) {
            return new Pose(this.x + other.x, this.y + other.y, this.theta + other.theta);
        }

        @NonNull
        public String toString() {
            return "x: " + x + System.lineSeparator() + "y: " + y + System.lineSeparator() + "theta: " + theta;
        }

        /**
         * Converts theta from radians to degrees.
         * @return a pose with radians converted to degrees
         */
        public Pose toDegrees() {
            return new Pose(this.x, this.y, Math.toDegrees(this.theta));
        }
    }

    public class PositionTracker {
        public Pose currentPos = new Pose(0, 0, 0);

        private int lastFL = 0;
        private int lastFR = 0;
        private int lastBL = 0;
        private int lastBR = 0;
        private double lastYaw = 0;

        /**
         * Override the current pose estimate.
         * @param pose Pose to relocalize to. x and y must be in inches, and theta must be in radians.
         */
        public void setPoseEstimate(Pose pose) {
            currentPos = pose.toTicks();
        }

        private double sinc(double x) {
            return x == 0 ? 1 : Math.sin(x) / x;
        }

        // this function doesn't really have a standard name, but it's similar to sinc so cosc it is
        // not to be confused with cosec
        private double cosc(double x) {
            return x == 0 ? 0 : (1 - Math.cos(x)) / x;
        }

        /**
         * Updates the current position estimate. The more you call this, the better.
         */
        public void update() {
            int newFL = fl.getCurrentPosition();
            int newFR = fr.getCurrentPosition();
            int newBL = bl.getCurrentPosition();
            int newBR = br.getCurrentPosition();
            double newYaw = getYaw();

            int flDiff = newFL - lastFL;
            int frDiff = newFR - lastFR;
            int blDiff = newBL - lastBL;
            int brDiff = newBR - lastBR;
            double yawDiff = angleDiff(newYaw, lastYaw);

            lastFL = newFL;
            lastFR = newFR;
            lastBL = newBL;
            lastBR = newBR;
            lastYaw = newYaw;

            double xDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.0);
            double yDiff = ((-flDiff + frDiff + blDiff - brDiff) / 4.0) * Math.tan(Math.toRadians(40));

            double expX = cosc(yawDiff);
            double expY = sinc(yawDiff);

            Pose posDiff = new Pose(yDiff * expX + xDiff * expY, yDiff * expY - xDiff * expX, yawDiff);
            posDiff = posDiff.rot(currentPos.theta);

            currentPos = currentPos.add(posDiff);

            drawPose(currentPos);
        }
    }
}