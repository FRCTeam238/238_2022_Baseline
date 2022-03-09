package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class FeederCommandWithColor extends Command {

    public int heldBallsNumber = 0;
    boolean lastStateBroken = true;
    boolean secondSensorBroken = true;
    boolean firstSensorBroken = true;

    boolean thirdSensorBroken = true;

    Feeder theFeeder = Robot.feeder;

    Shooter theShooter = Robot.shooter;
    LED led = Robot.led;

    public FeederCommandWithColor() {
        requires(theFeeder);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        led.setColor(1, 150, 0, 0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        feederLogicLoop();
    }

    public void feederLogicLoop() {
        int beginningNumber = 10;
        int endNumber = 80;
        firstSensorBroken = theFeeder.firstDetector.get();

        secondSensorBroken = theFeeder.secondDetector.get();

        thirdSensorBroken = theFeeder.thirdDetector.get();

        if (thirdSensorBroken == false) {
            if (theShooter.isShooting == false) {
                theFeeder.stop();
            }
        } else {
            if (firstSensorBroken == false) { // First sensor IS tripped
                if (secondSensorBroken == true) {// second sensor IS NOT tripped
                    // if (DriverStation.getAlliance() == Alliance.Red) {
                    //     if (RobotMap.FeederDevices.ballColor.getBlue() > RobotMap.FeederDevices.ballColor.getRed()) {
                    //         Logger.Debug("blue ball!");
                    //         theFeeder.stop();
                    //         Robot.theClearIntake.start();
                    //     } else {
                    //         theFeeder.up();
                    //     }
                    // } else {
                    //     if (RobotMap.FeederDevices.ballColor.getBlue() < RobotMap.FeederDevices.ballColor.getRed()) {
                    //         Logger.Debug("Red ball!");
                    //         theFeeder.stop();
                    //         Robot.theClearIntake.start();
                    //     } else {
                    //         theFeeder.up();
                    //     }
                    // }
                    preventWrongColor();
                } else {
                    theFeeder.up();
                }
            }
            if (secondSensorBroken == true && lastStateBroken == false) { // Secondq sensor is NOT tripped, but just WAS
                heldBallsNumber++;
                Logger.Debug("Held Balls Count = " + heldBallsNumber);
                endNumber = beginningNumber + (14 * heldBallsNumber);
                Color toSet = new Color(238, 238, 0);
                led.setColor(beginningNumber, endNumber, toSet);
                theFeeder.stop();
            }

            // activate LEDs here

            lastStateBroken = secondSensorBroken;

        }
        SmartDashboard.putBoolean("First Sensor", firstSensorBroken);
        SmartDashboard.putBoolean("Second Sensor", secondSensorBroken);
        SmartDashboard.putBoolean("Third Sensor", thirdSensorBroken);
    }

    public String getBallColor() {
        String colorOfBall;
        if (RobotMap.FeederDevices.ballColor.getBlue() > RobotMap.FeederDevices.ballColor.getRed()) {
            colorOfBall = "blue";
        } else {
            colorOfBall = "red";
        }
        return colorOfBall;
    }

    public void preventWrongColor() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            if (getBallColor() == "blue") {
                Logger.Debug("blue ball!");
                theFeeder.down();
                Robot.theClearIntake.start();

            } else {
                theFeeder.up();
            }
        } else {
            if (getBallColor() == "red") {
                Logger.Debug("red ball!");
                theFeeder.down();
                Robot.theClearIntake.start();
            } else {
                theFeeder.up();
            }

        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        heldBallsNumber = 0;
        // TODO change hard coded values
        led.setColor(1, 60, 0, 0, 0);
        theFeeder.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        theFeeder.stop();
    }
}