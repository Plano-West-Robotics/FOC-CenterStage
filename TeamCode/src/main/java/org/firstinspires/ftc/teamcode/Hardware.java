package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.inchworm.InchWorm;
import org.firstinspires.ftc.teamcode.poser.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * Hardware wrapper to abstract motors and stuff. I recommend you use this to keep yourself sane.
 */
public class Hardware {
    public DcMotorEx fl, fr, bl, br;
    public DcMotorEx intake, ramp;
    public DcMotorEx liftL, liftR;
    public Encoder leftOdo, backOdo, rightOdo;
    public Servo launcherPin, launcherBase;
    public Servo elbowL, elbowR;
    public Servo wristL, wristR;
    public Servo flap, blocker;
    public IMU imu;
    public VoltageSensor voltageSensor;

    public RevColorSensorV3 top, bottom;
    public RevBlinkinLedDriver ledLeft, ledRight;

    public RevTouchSensor slideLimitSwitch;

    public OpenCvCamera webcam;

    public OpMode opMode;
    public DashboardTelemetryWrapper dashboardTelemetry;

    /**
     * Initialize hardware wrapper. This constructor reverses motors and all that for you
     *
     * @param opMode Current opmode. Used to grab the HardwareMap
     */
    public Hardware(OpMode opMode) {
        this.opMode = opMode;

        HardwareMap hardwareMap = opMode.hardwareMap;

        this.dashboardTelemetry = new DashboardTelemetryWrapper(FtcDashboard.getInstance());
        opMode.telemetry = new MultipleTelemetry(opMode.telemetry, this.dashboardTelemetry);

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // TODO: Change this to a CRServo if needed
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        // commented out because if encoder isn't plugged in it causes this to run at full speed
//        ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ramp.setDirection(DcMotorSimple.Direction.REVERSE);

        liftL = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftR = hardwareMap.get(DcMotorEx.class, "liftRight");

//        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setDirection(DcMotorSimple.Direction.FORWARD);

        leftOdo = null; // TODO
        backOdo = new Encoder(ramp);
        rightOdo = new Encoder(intake);

        elbowL = hardwareMap.get(Servo.class, "elbowL");
        elbowR = hardwareMap.get(Servo.class, "elbowR");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        flap = hardwareMap.get(Servo.class, "flap");
        blocker = hardwareMap.get(Servo.class, "blocker");

        /*
        * Measured 2024-01-18
        *            in - out
        *  elbowR: 0.20 - 0.65
        *  elbowL: 0.77 - 0.32
        *  wristR: 0.31 - 0.51 (intermediate: 0.17)
        *  wristL: 0.61 - 0.41 (intermediate: 0.75)
        *
        *       unblock - block
        * blocker: 0.40 - 0.90
        *
        *        closed - open
        *    flap: 0.40 - 0.75
        */

        elbowR.scaleRange(0.20, 0.65);
        elbowR.setDirection(Servo.Direction.FORWARD);
        elbowL.scaleRange(0.32, 0.77);
        elbowL.setDirection(Servo.Direction.REVERSE);
        wristR.scaleRange(0.11, 0.51);
        wristR.setDirection(Servo.Direction.FORWARD);
        wristL.scaleRange(0.41, 0.81);
        wristL.setDirection(Servo.Direction.REVERSE);
        blocker.scaleRange(0.40, 0.90);
        flap.scaleRange(0.40, 0.75);

        launcherPin = hardwareMap.get(Servo.class, "launcherPin");
        launcherBase = hardwareMap.get(Servo.class, "launcherBase");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(InchWorm.GLOBAL_ORIENTATION));

        top = hardwareMap.get(RevColorSensorV3.class, "top");
        bottom = hardwareMap.get(RevColorSensorV3.class, "bottom");

        ledLeft = hardwareMap.get(RevBlinkinLedDriver.class, "ledLeft");
        ledRight = hardwareMap.get(RevBlinkinLedDriver.class, "ledRight");

        slideLimitSwitch = hardwareMap.get(RevTouchSensor.class, "slideLimitSwitch");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
    }

    /**
     * Get yaw in specified unit. Counterclockwise is positive. The zero position is defined by a call to <code>resetYaw</code>
     *
     * @param angleUnit Unit of the returned angle
     * @return yaw in specified unit
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Reset yaw reading to 0.
     */
    public void resetYaw() {
        imu.resetYaw();
    }

    /**
     * Get yaw in radians. Counterclockwise is positive. The zero position is defined by a call to <code>resetYaw</code>.
     *
     * @return yaw in radians
     */
    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }
}
