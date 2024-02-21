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
    public enum FlapPosition {
        CLOSED(0), OPEN(1);

        private final double pos;

        FlapPosition(double p) {
            pos = p;
        }
    }

    public enum BlockerPosition {
        BLOCK(1), UNBLOCK(0); // todo: change these if needed

        private final double pos;

        BlockerPosition(double p) {
            pos = p;
        }
    }

    public ArmPosition currentArmPos = ArmPosition.DOWN;
    public FlapPosition currentFlapPos = FlapPosition.OPEN;
    public BlockerPosition currentBlockerPos = BlockerPosition.UNBLOCK;
    Hardware hardware;

    public Arm(Hardware hw) {
        hardware = hw;
    }

    public void setArmPosition(ArmPosition newPos) {
        currentArmPos = newPos;

        hardware.elbowL.setPosition(newPos.elbowPos);
        hardware.elbowR.setPosition(newPos.elbowPos);

        hardware.wristL.setPosition(newPos.wristPos);
        hardware.wristR.setPosition(newPos.wristPos);
    }

    public void setFlapPosition(FlapPosition newPos) {
        currentFlapPos = newPos;

        hardware.flap.setPosition(newPos.pos);
    }

    public void toggleFlapPosition() {
        switch (currentFlapPos) {
            case CLOSED:
                setFlapPosition(FlapPosition.OPEN);
                break;
            case OPEN:
                setFlapPosition(FlapPosition.CLOSED);
                break;
        }
    }

    public void setBlockerPosition(BlockerPosition newPos) {
        currentBlockerPos = newPos;

        hardware.blocker.setPosition(newPos.pos);
    }

    public void toggleBlockerPosition() {
        switch (currentBlockerPos) {
            case BLOCK:
                setBlockerPosition(BlockerPosition.UNBLOCK);
                break;
            case UNBLOCK:
                setBlockerPosition(BlockerPosition.BLOCK);
                break;
        }
    }
}
