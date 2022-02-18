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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterController;
    //private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterMaster;
    private CANSparkMax shooterFollowerDrive = RobotMap.ShooterDevices.shooterFollower;
    //private final int followerID = 17;
    //NOT THE REAL ID
    private SparkMaxPIDController shooterPID;
    private RelativeEncoder shooterEncoder;

    //2021 pid values
    private double kP = 0.0002;//0.00005;
    private double kI = 0;
    private double kD = 0.001;//0.09;
    private double kIZ = 0;
    private double kFF = 1.8e-4;
    private double kMinOutput = 0;
    private double kMaxOutput = 12;

    // private double kP = 8.8253E-10;
    // private double kI = 0;
    // private double kD = 0;
    // private double kIZ = 0;
    // private double kFF = 0;
    // private double kMinOutput = 0;
    // private double kMaxOutput = 12;

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

    public boolean isShooting = false;

    SimpleMotorFeedforward simpleMotorFeedforward;

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
        dashboard = Robot.dashboard238;
        entry = Shuffleboard.getTab("DiagnosticTab").add("Shooter At Speed", false).getEntry();
        simpleMotorFeedforward = new SimpleMotorFeedforward(RobotMap.ShooterDevices.SHOOTER_ks, RobotMap.ShooterDevices.SHOOTER_kv); 
    }

    public void initSparkMax() {
        shooterMasterDrive.restoreFactoryDefaults();
        shooterMasterDrive.setInverted(false);
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
        double feedForward = simpleMotorFeedforward.calculate(speedValue);
        //replace below getPIDController to match arbitrarty feed forward

        desiredSpeedPID = speedValue;
        //shooterMasterDrive.getPIDController().setReference(desiredSpeedPID, CANSparkMax.ControlType.kVelocity, 0, feedForward);
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
        double currentSpeed = getSpeed();
        double speedDifference = currentSpeed - desiredSpeedPID;
        if(Math.abs(speedDifference) < RobotMap.ShooterDevices.SHOOTER_SPEED_TOLERANCE){
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
