/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Hanger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.HangerDevices;

public class LowerHanger extends CommandBase {

  // only accept value if x is within low range and if y is outside of small
  // window

  Hanger theHanger = Robot.hanger;

  public LowerHanger() {
    addRequirements(theHanger);
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
      theHanger.raiseLower(RobotMap.HangerDevices.hangerDownSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    //done retracting when bottom limit switch is tripped
    return HangerDevices.downLimitSwitch.get() == false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    theHanger.brake();
    Logger.Debug("done lowering");
    if (interrupted) {
      theHanger.brake();
    }
  }
}
