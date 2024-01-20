package org.firstinspires.ftc.teamcode.macro;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * The MoveServo class moves a servo. 'nuff said
 */
public class MoveServo implements Action {
    private final Servo servo;
    private final double pos;

    public MoveServo(Servo servo, double pos) {
        this.servo = servo;
        this.pos = pos;
    }

    public ControlFlow update() {
        return ControlFlow.BREAK;
    }

    public void end() {
        servo.setPosition(pos);
    }

    public void run() {
        this.end();
    }
}