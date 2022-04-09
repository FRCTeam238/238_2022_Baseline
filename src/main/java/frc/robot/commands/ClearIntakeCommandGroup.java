// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClearIntakeCommandGroup extends CommandGroup {

  public ClearIntakeCommandGroup() {
    addSequential(new Delay(0.5));
    addSequential(new ClearIntake());
  }
}
