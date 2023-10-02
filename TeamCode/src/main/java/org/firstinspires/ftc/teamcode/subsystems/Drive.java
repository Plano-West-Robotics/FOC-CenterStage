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

        double flPower = (powerY - powerX + turn) * getSpeed();
        double frPower = (powerY + powerX - turn) * getSpeed();
        double blPower = (powerY + powerX + turn) * getSpeed();
        double brPower = (powerY - powerX - turn) * getSpeed();

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl, fr, bl, br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        double volComp = getVoltageCompensation();

        hardware.fl.setPower(flPower * volComp);
        hardware.fr.setPower(frPower * volComp);
        hardware.bl.setPower(blPower * volComp);
        hardware.br.setPower(brPower * volComp);
    }

    public double getVoltageCompensation() {
        double voltage = hardware.voltageSensor.getVoltage();
        return 12 / voltage;
    }
}
