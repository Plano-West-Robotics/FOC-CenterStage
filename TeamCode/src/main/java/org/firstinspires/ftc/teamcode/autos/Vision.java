package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;

public class Vision {
    FreeSight freeSight;

    public Vision(Hardware hardware, Alliance alliance) {
        this.freeSight = new FreeSight(hardware, hardware.opMode.telemetry, false);
        this.freeSight.init();
        this.freeSight.pipe.colorState = alliance.isRed() ? FreeSightPipeline.Prop.MAGENTA : FreeSightPipeline.Prop.BLUE;
    }

    public FreeSightPipeline.Side getSide() {
        return this.freeSight.getPosition();
    }

    public FreeSightPipeline.Side end() {
        this.freeSight.stop();
        FreeSightPipeline.Side side = freeSight.getPosition();
        if (side == null) side = FreeSightPipeline.Side.RIGHT;
        return side;
    }
}
