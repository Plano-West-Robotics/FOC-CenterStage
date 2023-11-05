package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class Arm {
    public enum ArmPosition {
        UP(0, 0),
        DOWN(1, 1);
        private final double elbow, wrist;

        ArmPosition(double elbow, double wrist) {
            this.elbow = elbow;
            this.wrist = wrist;
        }
    }
    public enum PegPosition {
        ENGAGED(1), DISENGAGED(0);

        private final double pos;

        PegPosition(double p) {
            pos = p;
        }
    }

    public ArmPosition currentArmPos = ArmPosition.DOWN;
    public PegPosition currentPegPos = PegPosition.DISENGAGED;
    Hardware hardware;
    Telemetry telemetry;
    public Arm(Hardware hw, Telemetry telem) {
        hardware = hw;
        telemetry = telem;
    }

    public void setArmPosition(ArmPosition newPos) {
        currentArmPos = newPos;

        hardware.elbowL.setPosition(newPos.elbow); // todo: test this and see
        hardware.elbowR.setPosition(1-newPos.elbow);

        hardware.wristL.setPosition(newPos.wrist); // todo: test this and see
        hardware.wristR.setPosition(1-newPos.wrist);
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

    public void setPegPosition(PegPosition newPos) {
        currentPegPos = newPos;

        hardware.peg.setPosition(newPos.pos);
    }

    public void togglePegPosition() {
        switch (currentPegPos) {
            case ENGAGED:
                setPegPosition(PegPosition.DISENGAGED);
                break;
            case DISENGAGED:
                setPegPosition(PegPosition.ENGAGED);
                break;
        }
    }
}
