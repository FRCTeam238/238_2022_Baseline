/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.NavigationBoard;

public class TurnByNavigationBoard extends BaseTurn {
  private NavigationBoard navigationBoard;

  public TurnByNavigationBoard() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    super.initialize();
    //requires(Robot.navigationBoard);

    this.navigationBoard = Robot.navigationBoard;

    navigationBoard.zeroYaw();
    navigationBoard.setTargetYaw(angle);
  }

  @Override
  protected double getOffset(){
    return angle - navigationBoard.getYaw();
  }

  @Override
  public boolean isFinished(){
    return navigationBoard.isAtTargetYaw();
  }
}
