/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class TraversalSequence extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */

  
  public TraversalSequence() {
    addRequirements(Robot.hanger);
    
    addCommands(new LowerHanger()); //pull onto mid bar
    addCommands(new TraversalExtendCommand()); //extend traversal to put passive hooks above bar
    addCommands(new WaitCommand(1.25)); //1.34 //1.25          //delay to let passive hooks move. Change from '1' to match needed time
    addCommands(new RaiseHanger());         //raise hanger towards high bar
    addCommands(new TraversalRetractCommand()); //retract traversal to pull extended hook into high bar
    addCommands(new WaitCommand(1));            //delay to let passive hooks rotate robot. Change time if needed
    addCommands(new LowerHanger());           //Pull onto high bar
    addCommands(new TraversalExtendCommand()); //extend traversal to put passive hooks above high bar
    addCommands(new WaitCommand(1));          //delay to let passive hooks move. Change from '1' to match needed time
    addCommands(new RaiseHanger());         //raise hanger towards traversal bar
    addCommands(new TraversalRetractCommand()); //retract traversal to pull extended hook into traversal bar
    addCommands(new WaitCommand(1));            //delay to let passive hooks rotate robot. Change time if needed
    addCommands(new LowerHanger());           //Pull onto traversal bar
    addCommands(new TraversalExtendCommand()); //extend traversal to put passive hooks above traversal bar

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addSequential()
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
