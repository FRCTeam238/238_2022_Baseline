/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SetShooterSpeedCommand extends CommandBase {
  private double speed = -1;

  public SetShooterSpeedCommand(double speed) {
    addRequirements(Robot.shooter);
    this.speed = speed;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(Robot.shooter.getDesiredSpeed() == speed){
      Robot.shooter.neutral();
      SmartDashboard.putBoolean("SHOOTER SPINNING", false);
    }else{
      Robot.shooter.setSpeed(speed);
      SmartDashboard.putBoolean("SHOOTER SPINNING", true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Robot.shooter.neutral();
    }
  }
}
