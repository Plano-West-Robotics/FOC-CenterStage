package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.macro.Action;
import org.firstinspires.ftc.teamcode.macro.Macro;
import org.firstinspires.ftc.teamcode.macro.Sequence;
import org.firstinspires.ftc.teamcode.macro.SimpleExecutor;
import org.firstinspires.ftc.teamcode.macro.Wait;

public class ControlledArm {
    private Arm arm;

    private final SimpleExecutor waiter = new SimpleExecutor();

    private ArmPosition position;

    public enum ArmPosition {
        UP,
        DOWN,
        FIXEL,
    }

    public ControlledArm(Hardware hardware) {
        this.arm = new Arm(hardware);
        this.position = ArmPosition.DOWN;
    }

    public void moveUp() {
        switch (position) {
            case UP:
                return;
            case DOWN:
                waiter.run(Wait.millis(350));
                break;
            case FIXEL:
                waiter.run(Wait.millis(50));
                break;
        }
        arm.setArmPosition(Arm.ArmPosition.UP);
        position = ArmPosition.UP;
    }

    public void moveDown() {
        switch (position) {
            case DOWN:
                return;
            case UP:
                waiter.run(Wait.millis(350));
                break;
            case FIXEL:
                waiter.run(Wait.millis(350));
                break;
        }
        arm.setArmPosition(Arm.ArmPosition.DOWN);
        position = ArmPosition.DOWN;
    }

    public void moveToFixel() {
        switch (position) {
            case FIXEL:
                return;
            case UP:
                waiter.run(Wait.millis(50));
                break;
            case DOWN:
                waiter.run(Wait.millis(350));
                break;
        }
        arm.setArmPosition(Arm.ArmPosition.FIXEL);
        position = ArmPosition.FIXEL;
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

    public boolean isBusy() {
        return this.waiter.isRunning();
    }

    public ArmPosition getCurrentPosition() {
        return position;
    }

    public void update() {
        this.waiter.update();
    }
}
