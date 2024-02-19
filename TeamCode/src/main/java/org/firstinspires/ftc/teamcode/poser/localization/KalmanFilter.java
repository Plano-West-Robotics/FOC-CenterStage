package org.firstinspires.ftc.teamcode.poser.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.log.Plank;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;

public class KalmanFilter implements Localizer {
    Pose poseEstimate;
    public DeltaLocalizer input;
    FallibleLocalizer[] sensors;
    Angle justKillMeAlready;

    private double p = 1;

    private static final double Q = 0.4; // your model covariance
    private static final double R = 0.1; // your sensor covariance

    public KalmanFilter(Pose initialPose, DeltaLocalizer input, FallibleLocalizer... sensors) {
        this.poseEstimate = initialPose;
        this.input = input;
        this.sensors = sensors;

        justKillMeAlready = initialPose.yaw;
    }

    public void update() {
        // sourced from https://www.ctrlaltftc.com/advanced/the-kalman-filter (with modifications)

        Pose u = input.updateWithDelta();
        Plank foo = ((TwoDeadWheelLocalizer) input).hardware.log.chop("KalmanFilter");
        Distance2 bar = u.pos.rot(poseEstimate.yaw);
        foo.addData("posDiff", bar);
        poseEstimate = poseEstimate.then(u);
        justKillMeAlready = justKillMeAlready.add(u.yaw);

        p += Q;

        double k = p/(p + R);

        for (FallibleLocalizer sensor : sensors) {
            sensor.update();
            Pose z = sensor.getPoseEstimate();
            if (z == null) continue;

            Pose diff = z.sub(poseEstimate);
            diff = new Pose(diff.pos, diff.yaw.modSigned());
            poseEstimate = poseEstimate.then(diff.scale(k));
        }

        p = (1 - k) * p;

        foo.addData("fml", poseEstimate.yaw.valInDegrees());
    }

    @Override
    public Pose getPoseEstimate() {
        return new Pose(poseEstimate.pos, justKillMeAlready);
    }
}
