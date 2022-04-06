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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.core238.wrappers.SendableWrapper;
import frc.robot.Dashboard238;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
    private final CANSparkMax shooterMasterDrive = RobotMap.ShooterDevices.shooterController;
    private CANSparkMax shooterFollowerDrive = RobotMap.ShooterDevices.shooterFollower;

    private SparkMaxPIDController shooterPID;
    private RelativeEncoder shooterEncoder;

    // backspin??
    private final CANSparkMax backspinController = RobotMap.ShooterDevices.BackspinController;
    private SparkMaxPIDController backspinPID;
    private RelativeEncoder backspinEncoder;
    private double desiredBackspinSpeedPID = 0;

    // 2022 pid values
    private double kP = 4.8708E-05;// 1.2708E-06;//0.0002;//0.00005;
    private double kI = 0;
    private double kD = 0;// 0.001;//0.09;
    private double kIZ = 0;
    private double kFF = 0;// 1.8e-4;
    private double kMinOutput = 0;
    private double kMaxOutput = 12;

    private double desiredSpeedPID = 0;

    private double desiredPositionPID = 0;

    private NetworkTableEntry entry;

    private SimpleWidget highHubSpeedFromDashboard;
    private SimpleWidget lowHubSpeedFromDashboard;
    private SimpleWidget isTuningShooter;

    private boolean shooterAtSpeed = false;

    public boolean isShooting = false;

    SimpleMotorFeedforward simpleMotorFeedforward;

    Dashboard238 dashboard;

    public Shooter() {
        initSparkMax();
        dashboard = Robot.dashboard238;
        entry = Shuffleboard.getTab("DiagnosticTab").add("Shooter At Speed", false).getEntry();
        simpleMotorFeedforward = new SimpleMotorFeedforward(RobotMap.ShooterDevices.SHOOTER_ks,
                RobotMap.ShooterDevices.SHOOTER_kv);

        highHubSpeedFromDashboard = Shuffleboard.getTab("Shooter Tuning").add("Shooter high hub RPM",
                RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB);
        lowHubSpeedFromDashboard = Shuffleboard.getTab("Shooter Tuning").add("Shooter low hub RPM",
                RobotMap.ShooterDevices.SHOOTER_DEFAULT_LOW_HUB);
        isTuningShooter = Shuffleboard.getTab("Shooter Tuning").add("Tuning Shooter?", false);
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

        shooterEncoder.setPosition(0);
        shooterMasterDrive.setSmartCurrentLimit(40);
        shooterFollowerDrive.setSmartCurrentLimit(40);
        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIZ);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);

        // backspin??
        // replicated the settings for the normal shooter motors
        // should we set this inverted??
        backspinController.restoreFactoryDefaults();
        backspinController.setIdleMode(IdleMode.kCoast);
        backspinPID = backspinController.getPIDController();
        backspinEncoder = backspinController.getEncoder();

        backspinEncoder.setPosition(0);
        backspinController.setSmartCurrentLimit(40);
        backspinPID.setP(kP);
        backspinPID.setI(kI);
        backspinPID.setD(kD);
        backspinPID.setIZone(kIZ);
        backspinPID.setFF(kFF);
        backspinPID.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void initDefaultCommand() {
    }

    // set speed for one set of wheels
    public void setSpeed(double speedValue) {
        double feedForward = simpleMotorFeedforward.calculate(speedValue / 60);
        // replace below getPIDController to match arbitrarty feed forward

        desiredSpeedPID = speedValue;
        shooterMasterDrive.getPIDController().setReference(desiredSpeedPID, CANSparkMax.ControlType.kVelocity, 0,
                feedForward);
        // shooterMasterDrive.getPIDController().setReference(desiredSpeedPID,
        // CANSparkMax.ControlType.kVelocity);
    }

    // set speed 2.0 with ALL NEW BACKSPIN WHEEL TECHNOLOGY
    public void setSpeed(double speedValue, double backspinSpeedValue) {
        double feedForward = simpleMotorFeedforward.calculate(speedValue / 60);
        double backspinFeedForward = simpleMotorFeedforward.calculate(backspinSpeedValue / 60);

        // do backspin?? pid stuff
        desiredSpeedPID = speedValue;
        desiredBackspinSpeedPID = backspinSpeedValue;

        shooterMasterDrive.getPIDController().setReference(desiredSpeedPID, CANSparkMax.ControlType.kVelocity, 0,
                feedForward);
        backspinController.getPIDController().setReference(desiredBackspinSpeedPID, CANSparkMax.ControlType.kVelocity,
                0, backspinFeedForward);
    }

    public boolean isAtSpeed() {

        // get current speed
        // eval against desiredSpeedPID
        // allow for range
        boolean inRange = false;
        double currentSpeed = getSpeed();
        double speedDifference = currentSpeed - desiredSpeedPID;
        if (Math.abs(speedDifference) < RobotMap.ShooterDevices.SHOOTER_SPEED_TOLERANCE) {
            inRange = true;
        } else {
            inRange = false;
        }
        return inRange;
    }

    public double getSpeed() {
        double speed = shooterEncoder.getVelocity();
        return speed;
    }

    public double getDesiredSpeed() {
        return desiredSpeedPID;
    }

    public void setPosition(double desiredPosition) {
        shooterEncoder.setPosition(0);
        shooterPID.setReference(desiredPosition, CANSparkMax.ControlType.kPosition);
        desiredPositionPID = desiredPosition;
    }

    public double getPosition() {
        double position = shooterEncoder.getPosition();
        return position;
    }

    public double getDesiredPosition() {
        return desiredPositionPID;
    }

    public void neutral() {
        shooterPID.setReference(0, CANSparkMax.ControlType.kVoltage);
        desiredSpeedPID = 0;

        // backspin??
        backspinPID.setReference(0, CANSparkMax.ControlType.kVoltage);
        desiredBackspinSpeedPID = 0;
    }

    public boolean isTuningShooter() {
        return isTuningShooter.getEntry().getBoolean(false);
    }

    public double getHighSpeedFromDashboard() {
        double speed;
        if (isTuningShooter() == true) {
            speed = highHubSpeedFromDashboard.getEntry().getDouble(RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB);
        } else {
            speed = RobotMap.ShooterDevices.SHOOTER_DEFAULT_HIGH_HUB;
        }
        return speed;
    }

    public double getLowSpeedFromDashboard() {
        double speed;
        if (isTuningShooter() == true) {
            speed = lowHubSpeedFromDashboard.getEntry().getDouble(RobotMap.ShooterDevices.SHOOTER_DEFAULT_LOW_HUB);
        } else {
            speed = RobotMap.ShooterDevices.SHOOTER_DEFAULT_LOW_HUB;
        }
        return speed;
    }

    public void runShooterDiagnostics() {
        Shuffleboard.selectTab("DiagnosticTab");
        if (shooterAtSpeed) {
            neutral();
        } else {
            setSpeed(3000);
            if (getSpeed() >= 2900) {
                shooterAtSpeed = true;
            }
        }
        entry.setBoolean(shooterAtSpeed);
    }

    // yet ANOTHER way we can push values to the dashboard
    /*
     * private List<SendableWrapper> _sendables = new ArrayList<>();
     * private void addChild(String name, SendableWrapper wrapper){
     * _sendables.add(wrapper);
     * addChild(name, (Sendable)wrapper);
     * }
     * 
     * private void initLiveWindow() {
     * SendableWrapper power = new SendableWrapper(builder -> {
     * builder.addDoubleProperty("Power", this::getCurrent, null);
     * });
     * 
     * SendableWrapper speed = new SendableWrapper(builder -> {
     * builder.addDoubleProperty("Speed", this::getSpeed, null);
     * });
     * 
     * SendableWrapper desiredSpeed = new SendableWrapper(builder -> {
     * builder.addDoubleProperty("DesiredSpeed", this::getDesiredSpeed, null);
     * });
     * 
     * SendableWrapper actualPower = new SendableWrapper(builder -> {
     * builder.addDoubleProperty("actualPower", this::getPower, null);
     * });
     * 
     * addChild("Power", power);
     * addChild("Speed", speed);
     * addChild("DesiredSpeed", desiredSpeed);
     * addChild("Actual power", actualPower);
     * }
     */
}
