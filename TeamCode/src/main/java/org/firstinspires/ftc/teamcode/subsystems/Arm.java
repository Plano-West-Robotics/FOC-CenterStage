package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class Arm {
    public enum ArmPosition {
        FIXEL(1, 1),
        UP(0.894736842105263, 1),
        DOWN(0.31578947368421045, 0),
        INTERMEDIATE(0, 0);

        private final double wristPos;
        private final double elbowPos;

        ArmPosition(double wrist, double elbow) {
            this.wristPos = wrist;
            this.elbowPos = elbow;
        }
    }

    public ArmPosition currentArmPos = ArmPosition.DOWN;
    Hardware hardware;

    public Arm(Hardware hw) {
        hardware = hw;
        this.setArmPosition(ArmPosition.DOWN);
    }

    public void setArmPosition(ArmPosition newPos) {
        currentArmPos = newPos;

        hardware.elbowL.setPosition(newPos.elbowPos);
        hardware.elbowR.setPosition(newPos.elbowPos);

        hardware.wristL.setPosition(newPos.wristPos);
        hardware.wristR.setPosition(newPos.wristPos);
    }
}
