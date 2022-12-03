// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.core238.Logger;
import frc.core238.autonomous.AutonomousModeAnnotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
@AutonomousModeAnnotation(parameterNames = {"Message"})
public class AutoDebugMessage extends InstantCommand implements IAutonomousCommand{
  boolean isAuto = false;
  String debugMessage;
  
  @Override
  public boolean getIsAutonomousMode() {
    // TODO Auto-generated method stub
    return false;
  }

  public AutoDebugMessage() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.Debug(debugMessage);
  }

@Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    this.isAuto = isAutonomousMode;

  }

  @Override
  public void setParameters(List<String> parameters){
    debugMessage = parameters.get(0);
  }

  @Override
  public double getTimeout() {
      // TODO Auto-generated method stub
      return 0;
  }
}
