package org.firstinspires.ftc.teamcode.macro;

public class Sequence implements Action {
    private final Action[] actions;
    private int i;

    public static Action of(Action... actions) {
        return new Sequence(actions);
    }

    public Sequence(Action[] actions) {
        this.actions = actions;
        i = 0;
    }

    public ControlFlow update() {
        while (i < actions.length) {
            if (actions[i].update().shouldContinue()) {
                return ControlFlow.CONTINUE;
            } else {
                actions[i].end();
                i += 1;
            }
        }

        return ControlFlow.BREAK;
    }

    public void end() {
        if (i < actions.length) {
            actions[i].end();
        }
        i = 0;
    }

    public void run() {
        while (!Thread.interrupted() && i < actions.length) {
            actions[i].run();
            i += 1;
        }
    }
}
