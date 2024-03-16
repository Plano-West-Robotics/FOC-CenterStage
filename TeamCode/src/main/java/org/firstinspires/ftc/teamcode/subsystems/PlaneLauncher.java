package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.MoveCRServo;
import org.firstinspires.ftc.teamcode.macro.MoveServo;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;

public class PlaneLauncher {
    Hardware hardware;
    private static final double IDLE_POS = 0.55;
    private static final double AIM = 0.68;

    public PlaneLauncher(Hardware hw) {
        hardware = hw;
//        hardware.planeLauncher.setPosition(ENGAGED_POS);
    }

    public Action idle() {
        return new MoveServo(hardware.launcherBase, IDLE_POS);
    }

    public Action fire() {
        return Sequence.of(
                new MoveCRServo(hardware.launcherPin, -1),
                Wait.millis(500),
                new MoveCRServo(hardware.launcherPin, 0)
        );
    }

    public Action aim() {
        return new MoveServo(hardware.launcherBase, AIM);
    }
}
