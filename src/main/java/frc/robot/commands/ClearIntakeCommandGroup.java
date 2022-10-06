// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClearIntakeCommandGroup extends SequentialCommandGroup {

  public ClearIntakeCommandGroup() {
    addCommands(new Delay(0.5));
    addCommands(new ClearIntake());

  }
}
