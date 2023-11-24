package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

public class Arm {
    public enum ArmPosition {
        UP(1, 1),
        DOWN(0, 0);
        private final double elbow, wrist;

        ArmPosition(double elbow, double wrist) {
            this.elbow = elbow;
            this.wrist = wrist;
        }
    }
    public enum FlapPosition {
        ENGAGED(1), DISENGAGED(0);

        private final double pos;

        FlapPosition(double p) {
            pos = p;
        }
    }

    public ArmPosition currentArmPos = ArmPosition.DOWN;
    public FlapPosition currentFlapPos = FlapPosition.DISENGAGED;
    Hardware hardware;
    Telemetry telemetry;
    public Arm(Hardware hw, Telemetry telem) {
        hardware = hw;
        telemetry = telem;
    }

    public void setArmPosition(ArmPosition newPos) {
        currentArmPos = newPos;

        hardware.elbowL.setPosition(newPos.elbow); // todo: test this and see
        hardware.elbowR.setPosition(newPos.elbow);

        hardware.wristL.setPosition(newPos.wrist); // todo: test this and see
        hardware.wristR.setPosition(newPos.wrist);
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
            case ENGAGED:
                setFlapPosition(FlapPosition.DISENGAGED);
                break;
            case DISENGAGED:
                setFlapPosition(FlapPosition.ENGAGED);
                break;
        }
    }
}
