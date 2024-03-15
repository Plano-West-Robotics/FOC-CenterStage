package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeWrapper;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

// teleop for ripfips, doesn't have intake or lift or any other subsystems enabled

@TeleOp(name = "RIPFIP TeleOp")
public class FIPOp extends OpModeWrapper {
    double driveSpeed;
    Drive drive;
    @Override
    public void setup() {
        driveSpeed = 0.6;
        drive = new Drive(this.hardware, driveSpeed);
        drive.setFieldOriented(false);
    }

    @Override
    public void run() {
        driveSpeed = Range.clip(driveSpeed, 0.1, 1.0);
        drive.setSpeed(driveSpeed);
        telemetry.addData("Drive speed", driveSpeed);

        double y = gamepads.getAnalogValue(Controls.STRAIGHT);
        double x = gamepads.getAnalogValue(Controls.STRAFE);
        double turn = gamepads.getAnalogValue(Controls.TURN);
        drive.driveOld(x, y, turn);
    }
}
