package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.GamepadWrapper;

public abstract class OpModeWrapper extends OpMode {
    protected Hardware hardware;
    protected GamepadWrapper gamepad1;
    protected GamepadWrapper gamepad2;

    @Override
    public final void init() {
        this.hardware = new Hardware(this);
        this.gamepad1 = new GamepadWrapper(super.gamepad1);
        this.gamepad2 = new GamepadWrapper(super.gamepad2);

        this.setup();
    }

    public void setup() {}

    @Override
    public final void loop() {
        this.gamepad1.update(super.gamepad1);
        this.gamepad2.update(super.gamepad2);

        this.run();
    }

    public void run() {}
}
