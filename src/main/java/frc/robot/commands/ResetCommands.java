// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class ResetCommands extends InstantCommand implements IAutonomousCommand {

  boolean isAutonomousMode;
  /** Add your docs here. */
  public ResetCommands() {
    addRequirements(Robot.intake);
    // Use addRequirements() here to declare subsystem dependencies
    // eg. addRequirements(chassis);
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    IntakeInOutCommand.isDone = false;
    FeederCommand.isDone = false;
  }

  @Override
  public boolean getIsAutonomousMode() {
    return isAutonomousMode;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAutonomousMode = isAutonomousMode;
  }

  @Override
  public void setParameters(List<String> parameters) {
  }
}
