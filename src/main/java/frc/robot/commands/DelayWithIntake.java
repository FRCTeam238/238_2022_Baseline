// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    addRequirements(theFeeder);
    addRequirements(theIntake);

    InstantCommand instantCommand = new InstantCommand(() -> {
      IntakeInOutCommand.isDone = true;
      FeederCommand.isDone = true;
    });
    
    delayCommand = new Delay();
    addCommands(delayCommand);
    addCommands(instantCommand);

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
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
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setIsAutonomousMode(boolean isAutonomousMode) {
    // TODO Auto-generated method stub
    
  }
}
