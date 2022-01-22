/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import frc.robot.Robot;
import frc.robot.subsystems.Vision;

public class DriveStraightVision extends BaseDriveStraight implements IAutonomousCommand {
  private boolean isAutonomousMode;
  private int desiredDistanceFromTarget = 1;
  private Vision vision;

  public DriveStraightVision() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    super.initialize();

    //requires(Robot.vision);
    this.vision = Robot.vision;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return desiredDistanceFromTarget <= vision.getDistanceToTarget();
  }

  @Override
  protected double getOffset(){
    return vision.getYaw();
  }

  @Override
  public boolean getIsAutonomousMode() {
    return this.isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;

  }

  @Override
  public void setParameters(List<String> parameters) {
    this.desiredDistanceFromTarget = Integer.parseInt(parameters.get(0));
  }
}
