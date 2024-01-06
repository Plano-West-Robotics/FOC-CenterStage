package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class PlaneLauncher {
    Hardware hardware;
    private static final double LAUNCH_POS = 0;
    private static final double IDLE_POS = 0.45;
    private static final double AIM = 0.65;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public void idle() {
        hardware.launcherBase.setPosition(IDLE_POS);
    }

    public void fire() {
        hardware.launcherPin.setPosition(LAUNCH_POS);
    }

    public void aim() {
        hardware.launcherBase.setPosition(AIM);
    }
}
