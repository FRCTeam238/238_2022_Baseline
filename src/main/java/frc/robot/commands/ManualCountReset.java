package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualCountReset extends Command implements IAutonomousCommand{

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
    @Override
    public boolean getIsAutonomousMode() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void setIsAutonomousMode(boolean isAutonomousMode) {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void setParameters(List<String> parameters) {
        // TODO Auto-generated method stub
        
    }
    
}
