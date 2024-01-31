package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.poser.Angle;

public class Drive {
    Hardware hardware;
    private double speed;
    private boolean fieldOriented = false;
    private Angle yawOffset = Angle.ZERO;

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
    private void setYawOffset(Angle newOffset) {
        yawOffset = newOffset;
    }

    private Angle getYaw() {
        return Angle.inRadians(hardware.getYaw(AngleUnit.RADIANS)).sub(yawOffset);
    }

    public void resetYaw() {
        setYawOffset(getYaw().add(yawOffset));
    }

    public void drive(double x, double y, double turn) {
        double powerX, powerY;
        if (getFieldOriented()) {
            double yaw = getYaw().valInRadians();
            double sin = Math.sin(-yaw);
            double cos = Math.cos(-yaw);
            powerX = x * cos - y * sin;
            powerY = x * sin + y * cos;
        } else {
            powerX = x;
            powerY = y;
        }

        double flPower = (powerY + powerX + turn);
        double frPower = (powerY - powerX - turn);
        double blPower = (powerY - powerX + turn);
        double brPower = (powerY + powerX - turn);

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX))); // shortcut for max(abs([fl, fr, bl, br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        hardware.fl.setPower(flPower * speed);
        hardware.fr.setPower(frPower * speed);
        hardware.bl.setPower(blPower * speed);
        hardware.br.setPower(brPower * speed);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
