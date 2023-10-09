package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.teleop.Hardware;

public class Drive {
    Hardware hardware;
    private double speed = 1;
    private boolean fieldOriented = false;

    public Drive(Hardware hw) {
        hardware = hw;
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
            powerX = x * Math.cos(hardware.getYaw()) - y * Math.sin(hardware.getYaw());
            powerY = x * Math.sin(hardware.getYaw()) + y * Math.cos(hardware.getYaw());
        } else {
            powerX = x;
            powerY = y;
        }

        double volComp = getVoltageCompensation();

        double flPower = (powerY - powerX - turn) * volComp;
        double frPower = (powerY + powerX + turn) * volComp;
        double blPower = (powerY + powerX - turn) * volComp;
        double brPower = (powerY - powerX + turn) * volComp;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl, fr, bl, br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        hardware.fl.setPower(flPower * getSpeed());
        hardware.fr.setPower(frPower * getSpeed());
        hardware.bl.setPower(blPower * getSpeed());
        hardware.br.setPower(brPower * getSpeed());
    }

    public double getVoltageCompensation() {
        double voltage = hardware.voltageSensor.getVoltage();
        return 12 / voltage;
    }
}
