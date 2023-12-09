package org.firstinspires.ftc.teamcode.poser;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;

@TeleOp
public class DistanceTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hardware = new Hardware(this);
        Poser poser = new Poser(hardware, 0.9, false, Pose.ZERO);
        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);

        waitForStart();

        double lo = 0.5;
        double hi = 1.5;
        double mid = (lo + hi) / 2;
        Poser.Motion motion = poser.goTo(Distance.inTiles(mid), Distance.ZERO);
        while (true) {
            motion.update();
            gamepads.update(gamepad1, gamepad2);

            if (gamepads.justPressed(Gamepads.Button.GP1_DPAD_UP)) {
                lo = mid;
            } else if (gamepads.justPressed(Gamepads.Button.GP1_DPAD_DOWN)) {
                hi = mid;
            } else if (gamepads.justPressed(Gamepads.Button.GP1_CROSS)) {
                break;
            } else {
                continue;
            }

            mid = (lo + hi) / 2;
            motion = poser.goTo(Distance.inTiles(mid), Distance.ZERO);
            telemetry.addData("lo", lo);
            telemetry.addData("mid", mid);
            telemetry.addData("hi", hi);
            telemetry.addLine("Press dpad-up to move forward, dpad-down to move backward, A to move on.");
            telemetry.update();
        }
        motion.end();
        poser.goTo(Distance2.ZERO).run();
        double xAxisCalib = mid;

        telemetry.addLine("Realign the robot then press B.");
        telemetry.update();
        motion = poser.goTo(Distance2.ZERO);
        while (true) {
            motion.update();
            gamepads.update(gamepad1, gamepad2);
            if (gamepads.justPressed(Gamepads.Button.GP1_CIRCLE)) break;
        }
        motion.end();

        lo = 0.5;
        hi = 1.5;
        mid = (lo + hi) / 2;
        motion = poser.goTo(Distance.ZERO, Distance.inTiles(mid));
        while (true) {
            motion.update();
            gamepads.update(gamepad1, gamepad2);

            if (gamepads.justPressed(Gamepads.Button.GP1_DPAD_LEFT)) {
                lo = mid;
            } else if (gamepads.justPressed(Gamepads.Button.GP1_DPAD_RIGHT)) {
                hi = mid;
            } else if (gamepads.justPressed(Gamepads.Button.GP1_CROSS)) {
                break;
            } else {
                continue;
            }

            mid = (lo + hi) / 2;
            motion = poser.goTo(Distance.ZERO, Distance.inTiles(mid));
            telemetry.addData("lo", lo);
            telemetry.addData("mid", mid);
            telemetry.addData("hi", hi);
            telemetry.addLine("Press dpad-left to move left, dpad-right to move right, A to move on.");
            telemetry.update();
        }
        motion.end();
        poser.goTo(Distance2.ZERO).run();
        double yAxisCalib = mid;

        telemetry.addData("xAxisCalib", xAxisCalib);
        telemetry.addData("yAxisCalib", yAxisCalib);
        telemetry.addLine("Note: if the existing calib values are not 1, you should multiply them with the values just measured.");
        telemetry.update();
        while (!isStopRequested());
    }
}
