package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Vector2;

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

    /**
     * Inputs are interpreted in the OLD coordinate scheme.
     *
     * @param x right is positive
     * @param y forward is positive
     * @param turn cw is positive
     */
    public void driveOld(double x, double y, double turn) {
        this.drive(y, -x, -turn);
    }

    /**
     * Inputs are interpreted in the NEW coordinate scheme.
     *
     * @param x forward is positive
     * @param y left is positive
     * @param turn ccw is positive
     */
    public void drive(double x, double y, double turn) {
        Vector2 pow = new Vector2(x, y);
        if (getFieldOriented()) {
            pow = pow.rot(getYaw());
        }

        double flPower = (pow.x - pow.y - turn);
        double frPower = (pow.x + pow.y + turn);
        double blPower = (pow.x + pow.y - turn);
        double brPower = (pow.x - pow.y + turn);

        double scale = Math.max(1, (Math.abs(pow.x) + Math.abs(pow.y) + Math.abs(turn))); // shortcut for max(abs([fl, fr, bl, br]))
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
        driveOld(0, 0, 0);
    }
}
