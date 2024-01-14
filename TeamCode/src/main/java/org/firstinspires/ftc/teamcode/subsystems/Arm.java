package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class Arm {
    public enum ArmPosition {
        UP(1),
        DOWN(0);
        private final double wristPos;

        ArmPosition(double wrist) {
            this.wristPos = wrist;
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
        BLOCK(0), UNBLOCK(1); // todo: change these if needed

        private final double pos;

        BlockerPosition(double p) {
            pos = p;
        }
    }

    public ArmPosition currentArmPos = ArmPosition.DOWN;
    public FlapPosition currentFlapPos = FlapPosition.OPEN;
    public BlockerPosition currentBlockerPos = BlockerPosition.UNBLOCK;
    Hardware hardware;
    Telemetry telemetry;
    public Arm(Hardware hw, Telemetry telem) {
        hardware = hw;
        telemetry = telem;
    }

    public void setArmPosition(ArmPosition newPos) {
        currentArmPos = newPos;

//        hardware.elbowL.setPosition(newPos.elbow); // todo: test this and see
//        hardware.elbowR.setPosition(newPos.elbow);

        hardware.wristL.setPosition(newPos.wristPos); // todo: test this and see
        hardware.wristR.setPosition(newPos.wristPos);
    }

    public void toggleArmPosition() {
        switch (currentArmPos) {
            case UP:
                setArmPosition(ArmPosition.DOWN);
                break;
            case DOWN:
                setArmPosition(ArmPosition.UP);
                break;
        }
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

    // call this during init or before start probably
    public void holdElbows() {
        hardware.elbowL.setPosition(0.4);
        hardware.elbowR.setPosition(0.6);
    }
}
