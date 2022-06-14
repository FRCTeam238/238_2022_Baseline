/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.Logger;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrainparameters.DrivetrainParameters;
import frc.robot.commands.drivetrainparameters.IDrivetrainParametersSource;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  private Drivetrain drivetrain;
  private IDrivetrainParametersSource parameterSource;
  private IDrivetrainParametersSource defaultParameterSource;
  private double driverSlowSpeedMultiplier;

  private boolean isStarted = false;
 
  public TankDrive(IDrivetrainParametersSource defaultParameterSource, Drivetrain drivetrain) {
    // added drivetrain as a constructor parameter as sim mode was returning Robot.drivetrain = null
    addRequirements(drivetrain);
    
    this.drivetrain = drivetrain;
    this.driverSlowSpeedMultiplier = RobotMap.DrivetrainControllers.driverSlowSpeedMultiplier;
    this.defaultParameterSource = defaultParameterSource;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  /* Called repeatedly when this Command is scheduled to run, default command for drive train, runs continuously
  By default execute() takes the values from the Joysticks as the parameter source.
  It then creates a DriveTrainParameters object named "parameters" and puts the values from parameterSource into it.
  Finally it pulls the individual values from the parameters object and inputs them into the drivetrain drive() method.
  */
  @Override
  public void execute() {
      IDrivetrainParametersSource source = parameterSource == null ? defaultParameterSource : parameterSource;
      DrivetrainParameters parameters = source.Get();
      if (Math.abs(parameters.Left) < RobotMap.DrivetrainControllers.deadBandZoneValue) {
        parameters.Left = 0;
      } 
      if (Math.abs(parameters.Right) < RobotMap.DrivetrainControllers.deadBandZoneValue) {
        parameters.Right = 0;
      }
      if (RobotMap.Joysticks.driverStickLeft.getTrigger() || RobotMap.Joysticks.driverStickRight.getTrigger()) {
        drivetrain.drive(parameters.Left*driverSlowSpeedMultiplier, parameters.Right*driverSlowSpeedMultiplier);
      }else{
        drivetrain.drive(parameters.Left, parameters.Right);// , parameters.Angle);
      }
     
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
  }

  public void setParameterSource(IDrivetrainParametersSource parameterSource){
    this.parameterSource = parameterSource;
  }

  public IDrivetrainParametersSource getParameterSource(){
    return this.parameterSource;
  }

  public void resetParameterSource(){
    this.parameterSource = null;
  }
}
