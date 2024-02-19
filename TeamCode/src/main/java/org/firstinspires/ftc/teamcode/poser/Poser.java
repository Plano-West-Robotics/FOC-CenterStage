package org.firstinspires.ftc.teamcode.poser;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.poser.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.poser.localization.KalmanFilter;
import org.firstinspires.ftc.teamcode.poser.localization.Localizer;
import org.firstinspires.ftc.teamcode.poser.localization.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.ControlFlow;
import org.openftc.apriltag.AprilTagDetectorJNI;

public class Poser {
    private final Hardware hardware;
    public final Localizer localizer;

    private double speed;
    private Pose lastTarget;
    private boolean flipped;

    private static Distance MAX_VEL = Distance.inMM(939.571); // mm/s
    private static Angle MAX_ANG_VEL = Angle.inDegrees(156.941); // deg/s

    private static final boolean ENABLE_DRAWING = true;

    public Poser(Hardware hardware, double speed, boolean flipped, Pose initialPose) {
        this.hardware = hardware;
        AprilTagDetector detector1 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        AprilTagDetector detector2 = new AprilTagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11, 3, 3);
        this.localizer = new KalmanFilter(
                initialPose,
                new TwoDeadWheelLocalizer(hardware),
                new AprilTagLocalizer(detector1, hardware, AprilTagLocalizer.Camera.FRONT),
                new AprilTagLocalizer(detector2, hardware, AprilTagLocalizer.Camera.REAR)
        );
//        this.localizer = new Localizer.FromDelta(new TwoDeadWheelLocalizer(hardware), initialPose);
        this.speed = speed;
        // flip it back if needed
        // TODO: cursed
        this.lastTarget = flipped ? initialPose.flippedAcrossXAxis() : initialPose;
        this.flipped = flipped;

        if (ENABLE_DRAWING) {
            this.hardware.dashboardTelemetry.drawRobot(initialPose);
            this.hardware.dashboardTelemetry.update();
        }
    }

    public double getSpeed() {
        return this.speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void move(double powerX, double powerY, double turn) {
        double flPower = powerX - powerY - turn;
        double frPower = powerX + powerY + turn;
        double blPower = powerX + powerY - turn;
        double brPower = powerX - powerY + turn;

        this.hardware.fl.setPower(flPower);
        this.hardware.fr.setPower(frPower);
        this.hardware.bl.setPower(blPower);
        this.hardware.br.setPower(brPower);
    }

    private static void drawLine(Canvas canvas, Distance2 p1, Distance2 p2) {
        canvas.strokeLine(p1.x.valInInches(), p1.y.valInInches(), p2.x.valInInches(), p2.y.valInInches());
    }

    private static void drawLineDelta(Canvas canvas, Distance2 p1, Distance2 del) {
        drawLine(canvas, p1, p1.add(del));
    }

    public Motion moveBy(Distance x, Distance y) {
        return this.moveBy(new Distance2(x, y));
    }

    public Motion moveBy(Distance2 pos) {
        return this.moveBy(pos, Angle.ZERO);
    }

    public Motion moveBy(Angle yaw) {
        return this.moveBy(Distance2.ZERO, yaw);
    }

    public Motion moveBy(Distance x, Distance y, Angle yaw) {
        return this.moveBy(new Distance2(x, y), yaw);
    }

    public Motion moveBy(Distance2 pos, Angle yaw) {
        return this.moveBy(new Pose(pos, yaw));
    }

    public Motion moveBy(Pose pose) {
        return this.goTo(new Pose(
                this.lastTarget.pos.add(pose.pos),
                this.lastTarget.yaw.add(pose.yaw)
        ));
    }

    public Motion goToX(Distance x) {
        return this.goTo(x, lastTarget.pos.y);
    }

    public Motion goToY(Distance y) {
        return this.goTo(lastTarget.pos.x, y);
    }

    public Motion goTo(Distance x, Distance y) {
        return this.goTo(new Distance2(x, y));
    }

    public Motion goTo(Distance2 pos) {
        return this.goTo(pos, this.lastTarget.yaw);
    }

    public Motion goTo(Angle yaw) {
        return this.goTo(this.lastTarget.pos, yaw);
    }

    public Motion goTo(Distance x, Distance y, Angle yaw) {
        return this.goTo(new Distance2(x, y), yaw);
    }

    public Motion goTo(Distance2 pos, Angle yaw) {
        return this.goTo(new Pose(pos, yaw));
    }

    public Motion goTo(Pose pose) {
        this.lastTarget = pose;

        if (this.flipped) pose = pose.flippedAcrossXAxis();
        return this.new Motion(pose);
    }

    // should be in `Motion` but that's unsupported by the language
    private enum RotationDirection {
        CW,
        CCW,
        NONE,
    }

    public class Motion implements Action {
        protected final PIDController xCtrl = new PIDController(1.5, 0, 0.25);
        protected final PIDController yCtrl = new PIDController(1.5, 0, 0.25);
        protected final PIDController yawCtrl = new PIDController(1.5, 0, 0.15);
        protected Pose target;
        private RotationDirection rotationDirection;

        private long lastUpdate;

        public Motion(Pose target) {
            this.target = target;

            this.rotationDirection = RotationDirection.NONE;

            this.lastUpdate = System.nanoTime();
        }

        public Motion turningCw() {
            if (Poser.this.flipped) {
                this.rotationDirection = RotationDirection.CCW;
            } else {
                this.rotationDirection = RotationDirection.CW;
            }
            return this;
        }

        public Motion turningCcw() {
            if (Poser.this.flipped) {
                this.rotationDirection = RotationDirection.CW;
            } else {
                this.rotationDirection = RotationDirection.CCW;
            }
            return this;
        }

        public ControlFlow update() {
            Poser poser = Poser.this;

            if (ENABLE_DRAWING) {
                hardware.dashboardTelemetry.drawRobot(poser.localizer.getPoseEstimate());
                hardware.dashboardTelemetry.drawTarget(target, poser.localizer.getPoseEstimate());
                hardware.dashboardTelemetry.update();
            }

            long now = System.nanoTime();
            long dtNanos = now - lastUpdate;
            lastUpdate = now;
            double dt = dtNanos / (1000 * 1000 * 1000.);

            poser.localizer.update();
            Pose pose = poser.localizer.getPoseEstimate();

            Distance2 posError = target.pos.sub(pose.pos);
            Distance2 targetVel = new Distance2(xCtrl.update(posError.x), yCtrl.update(posError.y))
                    .rot(pose.yaw.neg());

            Angle angError = target.yaw.sub(pose.yaw);
            switch (rotationDirection) {
                case CW:
                    angError = angError.modNegative();
                    break;
                case CCW:
                    angError = angError.modPositive();
                    break;
                default:
                    angError = angError.modSigned();
            }
            Angle targetAngVel = yawCtrl.update(angError);

            Vector2 pow = targetVel.div(MAX_VEL);
            pow = pow.normalized().mul(Range.clip(pow.magnitude(), 0, 1) * poser.speed);
            pow = pow.rot(targetAngVel.mul(dt).div(2).neg()); // "reverse pose exp"
            double angPow = targetAngVel.div(MAX_ANG_VEL);
            angPow = Range.clip(angPow, -1, 1) * poser.speed;

            poser.move(pow.x, pow.y, angPow);

            return ControlFlow.continueIf(
                    posError.magnitude().valInMM() > 15 || Math.abs(angError.valInDegrees()) > 4
            );
        }

        public void end() {
            Poser.this.move(0, 0, 0);
            xCtrl.reset();
            yCtrl.reset();
            yawCtrl.reset();
        }
    }

    public class TuningMotion extends Motion {
        public TuningMotion(Pose target) {
            super(target);
        }

        public void setTarget(Pose target) {
            this.target = target;
        }

        public void setTransCoeffs(double kp, double ki, double kd) {
            this.xCtrl.kp = kp;
            this.xCtrl.ki = ki;
            this.xCtrl.kd = kd;

            this.yCtrl.kp = kp;
            this.yCtrl.ki = ki;
            this.yCtrl.kd = kd;
        }

        public void setYawCoeffs(double kp, double ki, double kd) {
            this.yawCtrl.kp = kp;
            this.yawCtrl.ki = ki;
            this.yawCtrl.kd = kd;
        }
    }
}