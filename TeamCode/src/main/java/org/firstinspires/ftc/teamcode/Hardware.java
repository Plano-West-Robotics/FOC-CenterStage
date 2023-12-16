package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

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
    public Servo armL, armR;
    public Servo flap, blocker;
    public Servo planeLauncher;
    public IMU imu;
    public VoltageSensor voltageSensor;
    public ColorSensor top;
    public ColorSensor bottom;
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

        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        flap = hardwareMap.get(Servo.class, "flap");
        blocker = hardwareMap.get(Servo.class, "blocker");

        /*
        * Measured 2023-12-30
        *           in    out
        * blocker: 0.40 - 0.75
        *    armR: 0.73 - 0.3
        *    armL: 0.13 - 0.58
        *    flap: 0.90 - 0.65
        */

        armR.scaleRange(0.3, 0.73);
        armL.scaleRange(0.13, 0.58);
        armR.setDirection(Servo.Direction.REVERSE);
        armL.setDirection(Servo.Direction.FORWARD);

        blocker.scaleRange(0.4, 0.75);

        flap.scaleRange(0.65, 0.9);

        planeLauncher = hardwareMap.get(Servo.class, "launcher");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(InchWorm.GLOBAL_ORIENTATION));

        top = hardwareMap.get(ColorSensor.class, "top");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");

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
