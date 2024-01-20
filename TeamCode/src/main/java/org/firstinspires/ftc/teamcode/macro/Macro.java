/**
 * The Macro class
 */
package org.firstinspires.ftc.teamcode.macro;

public class Macro {
    private boolean running;
    private final Action action;

    public Macro(Action action) {
        this.running = false;
        this.action = action;
    }

    public boolean isRunning() {
        return running;
    }

    public void start() {
        running = true;
    }

    public void stop() {
        if (running) action.end();
        running = false;
    }

    public void update() {
        if (running) {
            if (action.update().shouldStop()) {
                action.end();
                running = false;
            }
        }
    }
}
