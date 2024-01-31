package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.config.ConstantProvider;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;

@Autonomous
public class FSTunerOP extends LinearOpMode {
    @Override
    public void runOpMode()  {
        Hardware hardware = new Hardware(this);
        FreeSight vision = new FreeSight(hardware, this.telemetry, true);
        vision.init();


        waitForStart();
        while (opModeIsActive()) {
            this.telemetry.addData("highHSV", vision.pipe.highHSV);
            this.telemetry.addData("lowHSV", vision.pipe.lowHSV);
        }

        vision.stop();
    }
}
