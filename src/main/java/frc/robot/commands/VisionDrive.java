/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.drivetrainparameters.VisionParameterSource;
import frc.robot.subsystems.Vision;

/* 
  In intitalize(), getDefaultCommand() calls the execute for TankDrive command continuously.
  Then in execute(), visionPowerSource is input into the TankDrive object tankDrive as the parameterSource.
*/

public class VisionDrive extends Command {
  VisionParameterSource visionPowerSource;
  TankDrive tankDrive;
  Vision vision = Robot.vision;

  public VisionDrive() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    vision.ledsOn();
    vision.trackingMode();
    visionPowerSource = new VisionParameterSource();
    tankDrive = (TankDrive)Robot.drivetrain.getDefaultCommand();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    tankDrive.setParameterSource(visionPowerSource);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true or when cancelled or interrupted
  @Override
  protected void end() {
    tankDrive.resetParameterSource();
    //vision.ledsOff();
    //vision.cameraMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
