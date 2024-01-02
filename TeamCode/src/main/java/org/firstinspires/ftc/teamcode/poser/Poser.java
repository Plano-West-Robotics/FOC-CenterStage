package org.firstinspires.ftc.teamcode.poser;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

public class Poser {
    private final Hardware hardware;
    private final Localizer localizer;

    private double speed;
    private Pose lastTarget;
    private boolean flipped;

    private static final Distance MAX_VEL = Distance.inMM(965.522); // mm/s
    private static final Angle MAX_ANG_VEL = Angle.inDegrees(149.353); // deg/s

    public Poser(Hardware hardware, double speed, boolean flipped, Pose initialPose) {
        this.hardware = hardware;
        this.localizer = new TwoDeadWheelLocalizer(hardware, initialPose);

        this.speed = speed;
        // flip it back if needed
        // TODO: cursed
        this.lastTarget = flipped ? initialPose.flippedAcrossXAxis() : initialPose;
        this.flipped = flipped;

        drawRobotPose(initialPose);
    }

    private void move(double powerX, double powerY, double turn) {
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

    private static void drawRobotPose(Pose pose) {
        FtcDashboard db = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        canvas.setStroke("black");
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(
                pose.pos.x.valInInches() * (24 / 23.625),
                pose.pos.y.valInInches() * (24 / 23.625),
                9
        );
        canvas.strokeLine(
                pose.pos.x.valInInches() * (24 / 23.625),
                pose.pos.y.valInInches() * (24 / 23.625),
                pose.pos.x.valInInches() * (24 / 23.625) + 9 * pose.yaw.cos(),
                pose.pos.y.valInInches() * (24 / 23.625) + 9 * pose.yaw.sin()
        );

        db.sendTelemetryPacket(packet);
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

    public class Motion {
        private final Pose target;

        private long lastUpdate;
        private final PIDController xCtrl = new PIDController(1.5, 0, 0);
        private final PIDController yCtrl = new PIDController(1.5, 0, 0);
        private final PIDController yawCtrl = new PIDController(2.5, 0, 0);

        private boolean first;
        private Pose lastPose;
        private double runningXPow;
        private double runningYPow;
        private double runningAngPow;
        // TODO: tune
        private final PIDController xVelCtrl = new PIDController(1, 0, 0);
        private final PIDController yVelCtrl = new PIDController(1, 0, 0);
        private final PIDController angVelCtrl = new PIDController(1, 0 ,0);

        public Motion(Pose target) {
            this.target = target;

            this.lastUpdate = System.nanoTime();

            this.first = true;
        }

        public boolean update() {
            Poser poser = Poser.this;

            drawRobotPose(poser.localizer.getPoseEstimate());

            long now = System.nanoTime();
            long dtNanos = now - lastUpdate;
            lastUpdate = now;
            double dt = dtNanos / (1000 * 1000 * 1000.);

            poser.localizer.update();
            Pose pose = poser.localizer.getPoseEstimate();

            // position pid
            Distance2 posError = target.pos.sub(pose.pos);
            Distance2 targetVel = new Distance2(xCtrl.update(posError.x), yCtrl.update(posError.y))
                    .rot(pose.yaw.neg())
                    .mul(poser.speed);

            Angle angError = target.yaw.sub(pose.yaw).modSigned();
            Angle targetAngVel = yawCtrl.update(angError).mul(poser.speed);

            // velocity pid
            if (first) {
                runningXPow = targetVel.x.div(MAX_VEL);
                runningYPow = targetVel.y.div(MAX_VEL);
                runningAngPow = targetAngVel.div(MAX_ANG_VEL);
                first = false;
            } else {
                Distance2 currVel = pose.pos.sub(lastPose.pos).div(dt);
                Angle currAngVel = pose.yaw.sub(lastPose.yaw).div(dt);
                Distance xVelError = targetVel.x.sub(currVel.x);
                Distance yVelError = targetVel.y.sub(currVel.y);
                Angle angVelError = targetAngVel.sub(currAngVel);

                Distance xAccel = xVelCtrl.update(xVelError);
                Distance yAccel = yVelCtrl.update(yVelError);
                Angle angAccel = angVelCtrl.update(angVelError);

                runningXPow += xAccel.div(MAX_VEL) * dt;
                runningYPow += yAccel.div(MAX_VEL) * dt;
                runningAngPow += angAccel.div(MAX_ANG_VEL) * dt;
            }

            runningXPow = Range.clip(runningXPow, -1, 1);
            runningYPow = Range.clip(runningYPow, -1, 1);
            runningAngPow = Range.clip(runningAngPow, -1, 1);

            poser.move(runningXPow, runningYPow, runningAngPow);

            lastPose = pose;

            return posError.magnitude().valInMM() > 10 || Math.abs(angError.valInDegrees()) > 2;
        }

        public void end() {
            Poser.this.move(0, 0, 0);
        }

        public void run() {
            while (!Thread.interrupted() && this.update());
            this.end();
        }
    }
}