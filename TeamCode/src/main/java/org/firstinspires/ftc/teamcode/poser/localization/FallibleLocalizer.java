package org.firstinspires.ftc.teamcode.poser.localization;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;

public interface FallibleLocalizer {
    @Nullable
    Pose getPoseEstimate();

    void update();
}
