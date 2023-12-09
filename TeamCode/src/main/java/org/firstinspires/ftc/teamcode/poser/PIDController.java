package org.firstinspires.ftc.teamcode.poser;

public class PIDController {
    public final double p;
    public final double i;
    public final double d;

    private double integral;
    private double lastError;
    private double lastUpdate;
    private boolean firstIteration;

    public PIDController(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;

        this.reset();
    }

    public void reset() {
        this.firstIteration = true;
    }

    public double update(double error) {
        double now = System.nanoTime() / (1000 * 1000 * 1000.);

        double p = this.p * error;
        double i, d;

        if (this.firstIteration) {
            this.firstIteration = false;
            this.integral = 0;

            i = 0;
            d = 0;
        } else {
            double dt = now - this.lastUpdate;
            this.integral += error * dt;
            i = this.i * this.integral;
            d = this.d * (error - this.lastError) / dt;
        }

        this.lastUpdate = now;
        this.lastError = error;

        return p + i + d;
    }
}
