package org.firstinspires.ftc.teamcode.tune;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.opencv.core.Scalar;

public class ColorTuner extends OpModeWrapper {
    public static Scalar lowHSV = new Scalar(0,0,0);
    public static Scalar highHSV = new Scalar(255,255,255);

    @Override
    public void setup()
    {
//        FreeSightPipeline
    }

}
