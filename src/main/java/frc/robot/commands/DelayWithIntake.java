// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.core238.autonomous.AutonomousModeAnnotation;
import frc.robot.Robot;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

@AutonomousModeAnnotation(parameterNames = { "Timeout" })
public class DelayWithIntake extends SequentialCommandGroup implements IAutonomousCommand{

  public double timeout;
  Feeder theFeeder = Robot.feeder;
  Intake theIntake = Robot.intake;
  Delay delayCommand;

  public DelayWithIntake() {
    addCommands(new Delay(), 
      new InstantCommand(() -> {
        IntakeInOutCommand.isDone = true;
        FeederCommand.isDone = true;
      })
    );
  }

  @Override
  public void initialize() {
    delayCommand.timeout(timeout);
  }

  public void setParameters(List<String> parameters) {
    this.timeout = Double.parseDouble(parameters.get(0));
  }

  @Override
  public boolean getIsAutonomousMode() {
    return false;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {

  }
}
