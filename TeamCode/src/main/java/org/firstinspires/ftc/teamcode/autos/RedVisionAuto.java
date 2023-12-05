package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.freesight.client.FreeSight;
import org.firstinspires.ftc.teamcode.freesight.pipelines.FreeSightPipeline;
import org.firstinspires.ftc.teamcode.inchworm.InchWormBackup;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class RedVisionAuto extends LinearOpMode {
  @Override
  public void runOpMode() {
    Hardware hardware = new Hardware(this);
    FreeSight vision = new FreeSight(hardware, this.telemetry);
    vision.init();
    vision.pipe.colorState = FreeSightPipeline.Prop.PURPLE; // todo: change
    Intake intake = new Intake(hardware, 0.7);
    intake.stop();
    intake.reverse();
    InchWormBackup inchWorm = new InchWormBackup(this);

    FreeSightPipeline.Side currentSide = null;
    while (opModeInInit() && !opModeIsActive() && !isStopRequested()) {
      currentSide = vision.getPosition();
    }

    double angle = 0;
    double yOffset = 0;
    if (currentSide == null)
      currentSide = FreeSightPipeline.Side.MIDDLE;
    switch (currentSide) {
    case LEFT:
      yOffset = -2;
      angle = 60;
      break;
    case RIGHT:
      angle = -30;
      break;

    case MIDDLE:
    default:
      yOffset = 2;
      break;
    }

    inchWorm.moveTo(0, 27 + yOffset, Math.toRadians(angle));

    double time = getRuntime() + 1;
    intake.start();
    while (getRuntime() < time) {
      intake.update();
    }
    intake.stop();
    intake.update(); // so cursed bruhhh

    inchWorm.moveTo(-48, 5, 0); // todo: change y

    vision.stop();
  }
}
