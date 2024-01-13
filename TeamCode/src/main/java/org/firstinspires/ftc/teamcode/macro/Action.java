package org.firstinspires.ftc.teamcode.macro;

public interface Action {
    ControlFlow update();
    void end();

    default void run() {
        while (!Thread.interrupted() && this.update().shouldContinue());
        this.end();
    }

    static Action fromFn(Runnable runnable) {
        return new Oneshot(runnable);
    }

    class Oneshot implements Action {
        Runnable runnable;

        public Oneshot(Runnable runnable) {
            this.runnable = runnable;
        }

        public ControlFlow update() {
            return ControlFlow.BREAK;
        }

        public void end() {
            runnable.run();
        }

        public void run() {
            this.end();
        }
    }
}
