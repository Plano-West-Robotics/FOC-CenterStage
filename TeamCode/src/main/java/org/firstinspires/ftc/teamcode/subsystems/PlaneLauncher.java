package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class PlaneLauncher {
    Hardware hardware;
    private static final double LAUNCH_POS = 0;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public void fire() {
        hardware.launcherPin.setPosition(LAUNCH_POS);
    }

    public void aim() {
        hardware.launcherBase.setPosition(0.65);
    }
}
