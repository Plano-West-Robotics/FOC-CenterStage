/**
 * The Action class represents an action that the robot will take. The Actions will be used for
 * multi-step processes called Macros
 */
package org.firstinspires.ftc.teamcode.macro;

public interface Action {

    /**
     * If the Action should continue running
     * @return the state of run-age
     */
    ControlFlow update();

    /**
     * Stops the Action
     */
    void end();

    /**
     * The continued loop
     */
    default void run() {
        while (!Thread.interrupted() && this.update().shouldContinue());
        this.end();
    }

    /**
     * Generates an Action from a function
     * @param runnable the function
     * @return an Action to be ran
     */
    static Action fromFn(Runnable runnable) {
        return new Oneshot(runnable);
    }

    /**
     * A one time action created from a defined function
     */
    class Oneshot implements Action {
        Runnable runnable;

        /**
         * Constructs a Oneshot action
         * @param runnable the function to be ran
         */
        public Oneshot(Runnable runnable) {
            this.runnable = runnable;
        }

        /**
         * Stops the action
         * @return the stopped indicator
         */
        public ControlFlow update() {
            return ControlFlow.BREAK;
        }

        /**
         *
         */
        public void end() {
            runnable.run();
        }

        public void run() {
            this.end();
        }
    }
}
