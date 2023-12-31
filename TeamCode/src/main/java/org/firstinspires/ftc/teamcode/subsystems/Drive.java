package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hardware;

public class Drive {
    Hardware hardware;
    private double speed;
    private boolean fieldOriented = false;

    public Drive(Hardware hw, double speed) {
        this.hardware = hw;
        this.speed = speed;
    }

    public void setSpeed(double newSpeed) {
        speed = newSpeed;
    }

    public double getSpeed() {
        return speed;
    }

    public void setFieldOriented(boolean newFO) {
        fieldOriented = newFO;
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void toggleFieldOriented() {
        setFieldOriented(!getFieldOriented());
    }

    public void drive(double x, double y, double turn) {
        double powerX, powerY;
        if (getFieldOriented()) {
            double yaw = hardware.getYaw();
            double sin = Math.sin(-yaw);
            double cos = Math.cos(-yaw);
            powerX = x * cos - y * sin;
            powerY = x * sin + y * cos;
        } else {
            powerX = x;
            powerY = y;
        }

        double volComp = getVoltageCompensation();

        double flPower = (powerY + powerX + turn) * volComp;
        double frPower = (powerY - powerX - turn) * volComp;
        double blPower = (powerY - powerX + turn) * volComp;
        double brPower = (powerY + powerX - turn) * volComp;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(volComp)); // shortcut for max(abs([fl, fr, bl, br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        hardware.fl.setPower(flPower * speed);
        hardware.fr.setPower(frPower * speed);
        hardware.bl.setPower(blPower * speed);
        hardware.br.setPower(brPower * speed);
    }

    public double getVoltageCompensation() {
        double voltage = hardware.voltageSensor.getVoltage();
        return 12 / voltage;
    }
}
