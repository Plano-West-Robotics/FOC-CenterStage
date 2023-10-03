package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.freesight.opmodes.FreeSight;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp(name = "hopefully working teleop")
public class Teleop extends OpMode {
    Hardware hardware;
    Drive drive;

    @Override
    public void init() {
        hardware = new Hardware(this);
        drive = new Drive(hardware);
        FreeSight vision = new FreeSight(this);
        vision.findStuff();
        drive.setSpeed(0.5);
        drive.setFieldOriented(false);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad2.left_stick_x);
    }
}
