package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class PlaneLauncher {
    Hardware hardware;
    private static final double ENGAGED_POS = 0; // todo: change these later
    private static final double DISENGAGED_POS = 1;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public void disengage() {
        hardware.planeLauncher.setPosition(DISENGAGED_POS);
    }
}
