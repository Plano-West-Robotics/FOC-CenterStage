package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.MoveServo;

public class PlaneLauncher {
    Hardware hardware;
    private static final double LAUNCH_POS = 0;
    private static final double IDLE_POS = 0.2;
    private static final double AIM = 0.35;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public Action idle() {
        return new MoveServo(hardware.launcherBase, IDLE_POS);
    }

    public Action fire() {
        return new MoveServo(hardware.launcherPin, LAUNCH_POS);

    }

    public Action aim() {
        return new MoveServo(hardware.launcherBase, AIM);
    }
}
