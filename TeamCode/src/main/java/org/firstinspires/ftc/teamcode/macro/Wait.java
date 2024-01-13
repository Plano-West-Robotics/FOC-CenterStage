package org.firstinspires.ftc.teamcode.macro;

public class Wait implements Action {
    private boolean started;
    private long endTime;
    private final long duration;

    private static final long NANOS_PER_SEC = 1000 * 1000 * 1000;
    private static final double NANOS_PER_MILLI = 1000 * 1000;

    public static Wait seconds(double secs) {
        return new Wait((long)(secs * NANOS_PER_SEC));
    }

    public static Wait millis(double millis) {
        return new Wait((long)(millis * NANOS_PER_MILLI));
    }

    public Wait(long duration) {
        this.started = false;
        this.duration = duration;
    }

    public ControlFlow update() {
        if (!started) {
            started = true;
            endTime = System.nanoTime() + duration;
        } else {
            if (System.nanoTime() >= endTime) {
                return ControlFlow.BREAK;
            }
        }

        return ControlFlow.CONTINUE;
    }

    public void end() {
        started = false;
    }
}