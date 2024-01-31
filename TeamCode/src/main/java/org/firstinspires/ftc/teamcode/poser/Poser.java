package org.firstinspires.ftc.teamcode.poser;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.ControlFlow;

public class Poser {
    private final Hardware hardware;
    private final Localizer localizer;

    private double speed;
    private Pose lastTarget;
    private boolean flipped;

    private static Distance MAX_VEL = Distance.inMM(939.571); // mm/s
    private static Angle MAX_ANG_VEL = Angle.inDegrees(156.941); // deg/s

    private static final boolean ENABLE_DRAWING = false;

    public Poser(Hardware hardware, double speed, boolean flipped, Pose initialPose) {
        this.hardware = hardware;
        this.localizer = new TwoDeadWheelLocalizer(hardware, initialPose);

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
        private final PIDController xCtrl = new PIDController(2.5, 0, 0);
        private final PIDController yCtrl = new PIDController(2.5, 0, 0);
        private final PIDController yawCtrl = new PIDController(1.5, 0, 0);
        private final Pose target;
        private RotationDirection rotationDirection;

//        private long lastUpdate;

        public Motion(Pose target) {
            this.target = target;

            this.rotationDirection = RotationDirection.NONE;

//            this.lastUpdate = System.nanoTime();
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

//            long now = System.nanoTime();
//            long dtNanos = now - lastUpdate;
//            lastUpdate = now;
//            double dt = dtNanos / (1000 * 1000 * 1000.);

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
}