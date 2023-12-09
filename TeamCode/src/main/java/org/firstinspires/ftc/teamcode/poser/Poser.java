package org.firstinspires.ftc.teamcode.poser;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Hardware;

public class Poser {
    private Hardware hardware;
    private EncoderIntegrator localizer;

    private double speed;
    private Pose lastTarget;
    private boolean flipped;

    private static Distance MAX_VEL = Distance.inMM(1721.398 * EncoderIntegrator.MM_PER_ENCODER_TICK); // mm/s
    private static Angle MAX_ANG_VEL = Angle.inDegrees(149.353); // deg/s
    private static Distance MAX_ACCEL = MAX_VEL.div(1); // mm/s^2
    private static double MU = 0.2;

    public Poser(Hardware hardware, double speed, boolean flipped, Pose initialPose) {
        this.hardware = hardware;
        this.localizer = new EncoderIntegrator(hardware, initialPose);

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
        private final PIDController xCtrl = new PIDController(1.5, 0, 0);
        private final PIDController yCtrl = new PIDController(1.5, 0, 0);
        private final PIDController yawCtrl = new PIDController(2.5, 0, 0);

        private final Pose target;
        private Distance2 vel;
        private long lastUpdate;

        public Motion(Pose target) {
            this.target = target;

            this.vel = Distance2.ZERO;
            this.lastUpdate = System.nanoTime();
        }

        public boolean update() {
            Poser poser = Poser.this;

            drawRobotPose(poser.localizer.poseEstimate);

            long now = System.nanoTime();
            long dtNanos = now - this.lastUpdate;
            this.lastUpdate = now;
            double dt = dtNanos / (1000 * 1000 * 1000.);

            poser.localizer.update();

            // position
            Distance2 posError = this.target.pos.sub(poser.localizer.poseEstimate.pos);
//            double posErrorMM = posError.magnitude().valInMM();
//            double targetVelMM = Math.min(Math.sqrt(2 * MAX_ACCEL.valInMM() * posErrorMM) - 0.2 * MAX_ACCEL.valInMM(), MAX_VEL.valInMM());
//            Distance2 targetVel = posError.normalized().mul(Distance.inMM(targetVelMM));
//
//            Distance2 velError = targetVel.sub(vel);
//            Distance2 velDiff = velError.normalized().mul(
//                    Distance.inMM(Math.min(velError.magnitude().valInMM(), MAX_ACCEL.valInMM() * dt))
//            );
//
//            canvas.setStroke("orange");
//            canvas.strokeCircle(
//                    this.target.pos.x.valInInches(),
//                    this.target.pos.y.valInInches(),
//                    1
//            );
//            canvas.setStroke("black");
//            canvas.strokeCircle(
//                    poser.localizer.poseEstimate.pos.x.valInInches(),
//                    poser.localizer.poseEstimate.pos.y.valInInches(),
//                    9
//            );
//            drawLineDelta(canvas, poser.localizer.poseEstimate.pos, Distance2.inInches(0, 9).rot(poser.localizer.poseEstimate.yaw));
//            canvas.setStroke("red");
//            drawLineDelta(canvas, poser.localizer.poseEstimate.pos, targetVel);
//            canvas.setStroke("green");
//            drawLineDelta(canvas, poser.localizer.poseEstimate.pos, vel);
//            canvas.setStroke("blue");
//            drawLineDelta(canvas, poser.localizer.poseEstimate.pos.add(vel), velDiff);
//
//            db.sendTelemetryPacket(packet);
//
//            vel = vel.add(velDiff);
//            Vector2 pow =
//                    vel.div(MAX_VEL).mul(poser.speed).mul(MU)
//                            .add(velDiff.div(dt).div(MAX_ACCEL))
//                            .rot(poser.localizer.poseEstimate.yaw.neg());

            // angle
            Angle angError = this.target.yaw.sub(poser.localizer.poseEstimate.yaw).modSigned();
//            double angPow = Math.signum(angError.valInRadians()) * poser.speed;
//            if (angError.valInDegrees() < 60) {
//                angPow *= Math.pow(1 - Math.pow(1 - Math.abs(angError.valInDegrees() / 60), 1.6), 1/1.6);
//            }

            Distance2 pow = new Distance2(
                    Distance.inDefaultUnits(this.xCtrl.update(posError.x.valInDefaultUnits())),
                    Distance.inDefaultUnits(this.yCtrl.update(posError.y.valInDefaultUnits()))
            )
                    .rot(poser.localizer.poseEstimate.yaw.neg())
                    .mul(poser.speed);
            Angle angPow = Angle.inDefaultUnits(this.yawCtrl.update(angError.valInDefaultUnits()))
                    .mul(poser.speed);

            poser.move(pow.x.div(MAX_VEL), pow.y.div(MAX_VEL), angPow.div(MAX_ANG_VEL));

            return posError.magnitude().valInMM() > 10 || Math.abs(angError.valInDegrees()) > 2;
        }

        public void end() {
            Poser poser = Poser.this;

            poser.move(0, 0, 0);
            this.vel = Distance2.ZERO;
            this.lastUpdate = System.nanoTime();
        }

        public void run() {
            while (!Thread.interrupted() && this.update());
            this.end();
        }
    }
}