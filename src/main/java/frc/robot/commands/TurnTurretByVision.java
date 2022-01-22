/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurnTurretByVision extends Command {

  double wantedOrientation = 0; //In ticks
  Turret theTurret = Robot.turret;
  Vision vision = Robot.vision;
  public TurnTurretByVision() {
    requires(theTurret);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    vision.ledsOn();
    vision.trackingMode();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    findWantedOrientation();
    double desiredOutput = theTurret.limelightAngleConversion * wantedOrientation;
    theTurret.setVelocity(desiredOutput);
  }

  protected void findWantedOrientation(){
    wantedOrientation = vision.getYaw();
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    theTurret.stop();
    vision.cameraMode();
    vision.ledsOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
