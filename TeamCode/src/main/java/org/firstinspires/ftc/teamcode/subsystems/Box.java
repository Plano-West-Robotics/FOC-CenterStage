package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Box {
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

    public FlapPosition currentFlapPos = FlapPosition.OPEN;
    public BlockerPosition currentBlockerPos = BlockerPosition.UNBLOCK;
    Hardware hardware;

    public Box(Hardware hw) {
        hardware = hw;
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
