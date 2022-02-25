package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualCountReset extends Command {

    public ManualCountReset() {
    
    }
    @Override
    protected void execute() {
        Robot.feeder.resetBallCount();
    }
        
    
    
    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
    
}
