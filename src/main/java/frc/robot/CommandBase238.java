package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

/**
 * 238CommandBase
 */
public class CommandBase238 extends CommandGroupBase {

    private boolean isDone = false;
    private double timeToWait;
    private double initializedTime;
    public CommandBase238() {
        initializedTime = Timer.getFPGATimestamp();
    }    

    public double timeSinceInitialized() {
        return Timer.getFPGATimestamp() - initializedTime;
    }

    public void timeout(double time) {
        timeToWait = time;
    }

    public boolean hasTimedOut(){
        if (timeToWait >= timeSinceInitialized()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void addCommands(Command... commands) {
        // TODO Auto-generated method stub
        
    }

    
}