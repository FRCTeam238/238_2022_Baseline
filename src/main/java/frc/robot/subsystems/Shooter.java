/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.Logger;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    //private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    private CANSparkMax shooterFollowerDrive = RobotMap.ShooterDevices.shooterFollower;
    //private final int followerID = 17;
    //NOT THE REAL ID
    private SparkMaxPIDController shooterPID;
    private RelativeEncoder shooterEncoder;

    private double kP = 0.0008;//0.00005;
    private double kI = 0;
    private double kD = 0.09;
    private double kIZ = 0;
    private double kFF = 1.95e-4;
    private double kMinOutput = 0;
    private double kMaxOutput = 12;

    private double desiredSpeedPID = 0;
    private double desiredPositionPID = 0;

    public double shootTimePerBall = 1;

    public double ballsShot = 0;
    private double shooterVoltageAverage;
    private double samplesTaken = 1;
    private double totalVoltage = 0;
    private boolean hasBegunCounting = false;

    private double restartCounterTime = 0;

    private final double voltageSpikeSize = 2.2;
    private double ballCounterDelay = 0.75;

    public HashMap<Integer, Integer> distanceToShootMap = new HashMap<Integer, Integer>();

    private NetworkTableEntry entry;

    private boolean shooterAtSpeed = false;

    private double speedIncrease = 0;

    Dashboard238 dashboard;
    /*
     * private double integral = 0; private double derivative
     *  = 0; private double
     * previousError = 0; private double desiredSpeed = 0; private double
     * encoderTicks; private double previousEncoderTicks; private double error;
     * private double speed;
     */

    public Shooter() {
        initSparkMax();
        // initLiveWindow();
        populateSpeedMap(distanceToShootMap);
        dashboard = Robot.dashboard238;
        entry = Shuffleboard.getTab("DiagnosticTab").add("Shooter At Speed", false).getEntry();
        
    }

    public void initSparkMax() {
        shooterMasterDrive.restoreFactoryDefaults();
        shooterMasterDrive.setInverted(true);
        shooterFollowerDrive.restoreFactoryDefaults();
        shooterFollowerDrive.follow(shooterMasterDrive, true);
        shooterMasterDrive.setIdleMode(IdleMode.kCoast);
        shooterFollowerDrive.setIdleMode(IdleMode.kCoast);
        shooterPID = shooterMasterDrive.getPIDController();
        shooterEncoder = shooterMasterDrive.getEncoder();
        resetEncoder();
        shooterMasterDrive.setSmartCurrentLimit(40);
        shooterFollowerDrive.setSmartCurrentLimit(40);
        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIZ);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void initDefaultCommand() {
    }

    private void resetEncoder() {
        shooterEncoder.setPosition(0);
    }

    private double getEncoderTicks() {
        //GIVES IN ROTATIONS
        double encoderTicks = shooterEncoder.getPosition();
        return encoderTicks;
    }

    public void setSpeed(double speedValue) {
        desiredSpeedPID = speedValue;
        shooterMasterDrive.getPIDController().setReference(desiredSpeedPID, CANSparkMax.ControlType.kVelocity);
    }

    public void setPower(double power){
        if(Math.abs(power) > 0.15){
            double sign = ( power / (Math.abs(power)));
            power = sign*0.15;
        }
        shooterMasterDrive.set(power);
    }

    public boolean isAtSpeed() {

        //get current speed
        //eval against desiredSpeedPID
        // allow for range
        boolean inRange = false;
        double tolerance = 200;
        double currentSpeed = getSpeed();
        double speedDifference = currentSpeed - desiredSpeedPID;
        if(Math.abs(speedDifference) < tolerance){
            inRange = true;
        }else{
            inRange = false;
        }
        return inRange;
    }

    public void simpleSetSpeed(double speedValue){
        //shooterMasterDrive.set(speedValue);
    }


    //Changed from percent to raw
    public double getCurrent() {
        double power = shooterMasterDrive.getOutputCurrent();
        return power;
    }

    public double getPower(){
        double power = shooterMasterDrive.get();
        return power;
    }

    public double getSpeed() {
        double speed = shooterEncoder.getVelocity();
        return speed;
    }

    public double getDesiredSpeed(){
        return desiredSpeedPID;
    }

    public void setPosition(double desiredPosition){
        shooterEncoder.setPosition(0);
        shooterPID.setReference(desiredPosition, CANSparkMax.ControlType.kPosition);
        desiredPositionPID = desiredPosition;
    }

    public double getPosition(){
        double position = shooterEncoder.getPosition();
        return position;
    }

    public double getDesiredPosition(){
        return desiredPositionPID;
    }


    public void neutral() {
        shooterPID.setReference(0, CANSparkMax.ControlType.kVoltage);
        desiredSpeedPID = 0;
        stopCounting();
    }

    private void initLiveWindow() {
        SendableWrapper power = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Power", this::getCurrent, null);
        });

        SendableWrapper speed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("Speed", this::getSpeed, null);
        });

        SendableWrapper desiredSpeed = new SendableWrapper(builder -> {
            builder.addDoubleProperty("DesiredSpeed", this::getDesiredSpeed, null);
        });

        SendableWrapper actualPower = new SendableWrapper(builder -> {
            builder.addDoubleProperty("actualPower", this::getPower, null);
        });

        addChild("Power", power);
        addChild("Speed", speed);
        addChild("DesiredSpeed", desiredSpeed);
        addChild("Actual power", actualPower);
    }

    private List<SendableWrapper> _sendables = new ArrayList<>();
    private void addChild(String name, SendableWrapper wrapper){
      _sendables.add(wrapper);
      addChild(name, (Sendable)wrapper); 
    }

    private void populateSpeedMap(HashMap map){
        // Format: (distance, speed)
        // Distance is in inches, speed is in controller-side RPM
        map.put(80, 5000);
        map.put(96, 3600);
        map.put(112, 3600);
        map.put(128, 3620);
        map.put(144, 3665);
        map.put(160, 3750);
        map.put(176, 3840);
        map.put(192, 3950);
        map.put(208, 4000);
        map.put(224, 4075);
        map.put(240, 4100);
        map.put(256, 4150);
        map.put(272, 4230);
        map.put(288, 4300);
        map.put(304, 4415);
        map.put(320, 4575);
        map.put(336, 4700);
        map.put(352, 4900);
        map.put(368, 5000);
        map.put(384, 5100);
        map.put(400, 5270);
    }

    public int readSpeedMap(int distance){
        int neededRPMS = 0;
        Logger.Debug("Distance: " + distance);
        int distanceRemainder = distance % 16;
        if(distanceRemainder == 0){   
            if(distanceToShootMap.containsKey(distance)){
                neededRPMS = distanceToShootMap.get(distance);
            }else{
                Logger.Debug("Shooter.java line 231 - key not found");
            }
            
        }else{
            neededRPMS = getSteppedRPMs(distance, distanceRemainder, distanceToShootMap);
        }
        double increasedRPMS = neededRPMS * (speedIncrease);
        Logger.Debug("Needed RPMS: " + neededRPMS);
        Logger.Debug("increasedRPMS: " + increasedRPMS);
        return (int) increasedRPMS;
    }

    private int getSteppedRPMs(int distance, int distanceRemainder, HashMap<Integer, Integer> map){
        int lowerDistance = distance - distanceRemainder;
        int upperDistance = distance + (16 - distanceRemainder);
        int lowerRPM = 0;
        int upperRPM = 0;
        if(distanceToShootMap.containsKey(lowerDistance)){
            lowerRPM = distanceToShootMap.get(lowerDistance);
            if(distanceToShootMap.containsKey(upperDistance)){
                upperRPM = distanceToShootMap.get(upperDistance);
            }
        }
        int steppedRPMs = 0;
        int rpmGap = Math.abs(upperRPM - lowerRPM);
        int rpmPerInch = rpmGap / 16;
        int rpmAdjustment = distanceRemainder * rpmPerInch;
        if(upperRPM < lowerRPM){
            steppedRPMs = lowerRPM - rpmAdjustment;
        }else{
            steppedRPMs = lowerRPM + rpmAdjustment;
        }
        return (int) steppedRPMs;
    }

    private int getClosestRPMs(int distance, int distanceRemainder, HashMap<Integer, Integer> map){
        int closestRPMs = 0;
        if(distanceRemainder <= 8){
            if(map.containsKey(distance - distanceRemainder)){
                closestRPMs = map.get(distance - distanceRemainder);
            }else{
                Logger.Debug("Shooter.java line 239 - key not found");
            }
        }else{
            if(map.containsKey(distance + (16 - distanceRemainder))){
                closestRPMs = map.get(distance + (16 - distanceRemainder));
            }else{
                Logger.Debug("Shooter.java line 245 - key not found");
            }
        }
        return closestRPMs;
    }

    /** Takes a sample of what bus voltage is */
    private double getShooterBusVoltage (){
        return shooterMasterDrive.getBusVoltage();
    }

    /** Adds a new sample to the average and computes it */
    private void computeAverage (double newVoltage){
        totalVoltage = totalVoltage + newVoltage;
        shooterVoltageAverage = totalVoltage / samplesTaken;
    }

    /** Counts balls leaving the shooter based on voltage spikes. Delays the counter between shots to prevent fluctuations */
    public void countBalls (){
        if(hasBegunCounting){
            if(Timer.getFPGATimestamp() <= restartCounterTime){
                return;
            }
            computeAverage(getShooterBusVoltage());
            samplesTaken++;
            double delta = Math.abs(shooterVoltageAverage - getShooterBusVoltage());
            if(delta >= voltageSpikeSize){
                ballsShot++;
                restartCounterTime = Timer.getFPGATimestamp() + ballCounterDelay;
            }
            SmartDashboard.putNumber("BALLS SHOT", ballsShot);
        }else{
            // Logger.Debug("Ball Counter waiting to start");
        }
    }

    /** Enables the ball counter - still requires countBalls() to be run periodically */
    public void beginCounting(){
        hasBegunCounting = true;
    }

    /** Hard disables the ball counter, even if countBalls() is running */
    public void stopCounting(){
        hasBegunCounting = false;
    }

    public void runShooterDiagnostics(){
        Shuffleboard.selectTab("DiagnosticTab");
        if(shooterAtSpeed){
            neutral();
        }else{
            setSpeed(3000);
            if(getSpeed() >= 2900){
                shooterAtSpeed = true;
            }
        }
        entry.setBoolean(shooterAtSpeed);
    }
    /** Multiplies RPM by a scalar */
    public void increaseSpeed(double increase){
        speedIncrease = increase;
    }
}
