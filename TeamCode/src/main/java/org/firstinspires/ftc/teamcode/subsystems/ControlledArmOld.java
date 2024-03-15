package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.Macro;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;

public class ControlledArmOld {
    private Arm arm;

    private final Macro moveUp;
    private final Macro moveDown;
    private final Macro moveToFixel;

    private ArmPosition position;

    public enum ArmPosition {
        UP,
        DOWN,
        FIXEL
    }

    public ControlledArmOld(Hardware hardware) {
        this.arm = new Arm(hardware);
        this.position = ArmPosition.DOWN;

        moveUp = new Macro(
                Sequence.of(
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.INTERMEDIATE)),
                        Wait.millis(100),
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.UP)),
                        Wait.millis(270)
                )
        );
        moveDown = new Macro(
                Sequence.of(
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.INTERMEDIATE)),
                        Wait.millis(270),
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.DOWN)),
                        Wait.millis(100)
                )
        );
        moveToFixel = new Macro(
                Sequence.of(
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.INTERMEDIATE)),
                        Wait.millis(300),
                        Action.fromFn(() -> arm.setArmPosition(Arm.ArmPosition.FIXEL)),
                        Wait.millis(300)
                )
        );
    }

    public void moveUp() {
        if (position == ArmPosition.UP) return;
        position = ArmPosition.UP;

        this.moveDown.stop();
        this.moveToFixel.stop();
        this.moveUp.start();
    }

    public void moveDown() {
        if (position == ArmPosition.DOWN) return;
        position = ArmPosition.DOWN;

        this.moveUp.stop();
        this.moveToFixel.stop();
        this.moveDown.start();
    }

    public void moveToFixel() {
        if (position == ArmPosition.FIXEL) return;
        position = ArmPosition.FIXEL;

        this.moveUp.stop();
        this.moveDown.stop();
        this.moveToFixel.start();
    }

    public void moveTo(ArmPosition position) {
        switch (position) {
            case UP:
                this.moveUp();
                break;
            case DOWN:
                this.moveDown();
                break;
            case FIXEL:
                this.moveToFixel();
                break;
        }
    }

    /**
     * Note: can leave the arm in an invalid, intermediate state
     */
    public void stop() {
        this.moveUp.stop();
        this.moveDown.stop();
        this.moveToFixel.stop();
    }

    public boolean isBusy() {
        return this.moveUp.isRunning() || this.moveDown.isRunning() || this.moveToFixel.isRunning();
    }

    public ArmPosition getCurrentPosition() {
        return position;
    }

    public void update() {
        this.moveUp.update();
        this.moveDown.update();
        this.moveToFixel.update();
    }
}
