package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class PlaneLauncher {
    Hardware hardware;
    private static final double LAUNCH_POS = 0;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public void disengage() {
        hardware.launcherPin.setPosition(LAUNCH_POS);
    }

    public void aim() {
        // todo: tune this maybe
        hardware.launcherBase.setPosition(0.5);
    }
}
