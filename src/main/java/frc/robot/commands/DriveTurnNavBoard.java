/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.DrivetrainControllers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavigationBoard;


public class DriveTurnNavBoard extends BaseTurn {

  private NavigationBoard theNavigationBoard;
  private Drivetrain theDrivetrain;
  private double desiredAngle;

  public DriveTurnNavBoard() {
    requires(theDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    super.initialize();

    theNavigationBoard = Robot.navigationBoard;
    theDrivetrain = Robot.drivetrain;
    theNavigationBoard.zeroYaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    theDrivetrain.drive(0, 0, 30);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  @Override
  double getOffset() {
    double offSet = theNavigationBoard.getYaw();

    // TODO Auto-generated method stub
    return offSet;
  }
}
