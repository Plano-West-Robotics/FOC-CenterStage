package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.Macro;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.Wait;

public class ControlledArm {
    public Arm arm;

    private final Macro moveUp;
    private final Macro moveDown;

    public ControlledArm(Hardware hardware) {
        this.arm = new Arm(hardware);

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
    }

    public void moveUp() {
        if (this.moveUp.isRunning() || arm.currentArmPos == Arm.ArmPosition.UP) return;

        this.moveDown.stop();
        this.moveUp.start();
    }

    public void moveDown() {
        if (this.moveDown.isRunning() || arm.currentArmPos == Arm.ArmPosition.DOWN) return;

        this.moveUp.stop();
        this.moveDown.start();
    }

    /**
     * Note: can leave the arm in an invalid, intermediate state
     */
    public void stop() {
        this.moveUp.stop();
        this.moveDown.stop();
    }

    public boolean isBusy() {
        return this.moveUp.isRunning() || this.moveDown.isRunning();
    }

    public void update() {
        this.moveUp.update();
        this.moveDown.update();
    }
}
