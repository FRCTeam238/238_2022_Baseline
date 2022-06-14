package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class ClearIntake extends CommandBase {

    public ClearIntake(){
        addRequirements(Robot.intake);
    }
   @Override
   public void initialize() {
   }

   @Override
   public void execute() {
       Robot.intake.out(RobotMap.IntakeDevices.outtakeSpeed, RobotMap.MecanumDevices.mecanumOutSpeed);
   }
   
    @Override
    public boolean isFinished() {
        return false;
    }
    
}


