package org.firstinspires.ftc.teamcode.poser.localization;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;

public interface Localizer extends FallibleLocalizer {
    @NonNull
    Pose getPoseEstimate();

    class FromDelta implements Localizer {
        private final DeltaLocalizer inner;
        private Pose poseEstimate;

        public FromDelta(DeltaLocalizer inner, Pose initialPose) {
            this.inner = inner;
            this.poseEstimate = initialPose;
        }

        public void update() {
            this.poseEstimate = this.poseEstimate.then(this.inner.updateWithDelta());
        }

        public Pose getPoseEstimate() {
            return poseEstimate;
        }
    }
}
