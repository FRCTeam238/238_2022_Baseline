/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.NavigationBoard;

public class DriveStraightNavBoard extends BaseDriveStraight {

  private NavigationBoard navigationBoard;

  public DriveStraightNavBoard() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  @Override
  protected void initialize() {
    super.initialize();

    this.navigationBoard = Robot.navigationBoard;
    // sets the yaw to zero, base class will query and adjust for yaw offset
    navigationBoard.zeroYaw();    
  }

  @Override
  protected double getOffset(){
    return navigationBoard.getYaw();
  }
}
