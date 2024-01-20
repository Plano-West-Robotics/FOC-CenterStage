package org.firstinspires.ftc.teamcode.macro;

/**
 * A dynamic boolean trench coat
 */
public enum ControlFlow {
    CONTINUE, BREAK;

    public boolean shouldContinue() {
        return this == ControlFlow.CONTINUE;
    }

    public boolean shouldStop() {
        return this == ControlFlow.BREAK;
    }

    public static ControlFlow continueIf(boolean cond) {
        return cond ? CONTINUE : BREAK;
    }

    public static ControlFlow stopIf(boolean cond) {
        return cond ? BREAK : CONTINUE;
    }
}