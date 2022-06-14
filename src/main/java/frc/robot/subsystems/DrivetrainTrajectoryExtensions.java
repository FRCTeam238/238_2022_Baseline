/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.core238.Logger;
import frc.robot.Robot;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeInOutCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * DriveTrainSubsystem
 */
public class DrivetrainTrajectoryExtensions extends Drivetrain {

  //private final WPI_TalonSRX leftMasterDrive = new WPI_TalonSRX(DEVICE_ID_LEFT_MASTER);
  //private final WPI_VictorSPX leftDriveFollower1 = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE);
  //private final WPI_VictorSPX leftDriveFollower2 = new WPI_VictorSPX(DEVICE_ID_LEFT_SLAVE_TWO);
  //private final WPI_TalonSRX rightMasterDrive = new WPI_TalonSRX(DEVICE_ID_RIGHT_MASTER);
  //private final WPI_VictorSPX rightDriveFollower1 = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE);
  //private final WPI_VictorSPX rightDriveFollower2 = new WPI_VictorSPX(DEVICE_ID_RIGHT_SLAVE_TWO);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerDrive, rightControllerDrive);

  private final NavigationBoard gyro = Robot.navigationBoard;
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  public DrivetrainTrajectoryExtensions() {
      super();
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));    
/*
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 1023.0 / 6800.0;
    talonConfig.slot0.kP = 1.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = .25;

    rightMasterDrive.configAllSettings(talonConfig);
    leftMasterDrive.configAllSettings(talonConfig);

    leftMasterDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMasterDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    setNeutralMode(NeutralMode.Brake);

    rightMasterDrive.setInverted(true);
    rightDriveFollower1.setInverted(true);
    rightDriveFollower2.setInverted(true);
    rightMasterDrive.setSensorPhase(true);
    leftMasterDrive.setSensorPhase(true);
    rightMasterDrive.overrideLimitSwitchesEnable(false);
    leftMasterDrive.overrideLimitSwitchesEnable(false);

    leftDriveFollower1.follow(leftMasterDrive);
    leftDriveFollower2.follow(leftMasterDrive);
    rightDriveFollower1.follow(rightMasterDrive);
    rightDriveFollower2.follow(rightMasterDrive);
 */
    //differentialDrive.setRightSideInverted(false); done in DriveTrain now
   

    resetOdometry();
    setUseDifferentialDrive(false);
  }

  public void resetOdometry() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
    differentialDriveOdometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        stepsToMeters((int)getLeftEncoderPosition()),
        stepsToMeters((int)getRightEncoderPosition()));
    // SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed    speed along the x axis [-1.0..1.0]
   * @param rotation rotation rate along the z axis [-1.0..1.0]
   */
  public void arcadeDrive(double speed, double rotation) {
    arcadeDrive(speed, rotation, false);
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0]
   * @param rotation   rotation rate along the z axis [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    differentialDrive.arcadeDrive(speed, rotation, useSquares);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed, useSquares);
  }

  public void setUseDifferentialDrive(boolean useDifferentialDrive) {
    differentialDrive.setSafetyEnabled(useDifferentialDrive);
    if (!useDifferentialDrive) {
      // leftDriveFollower1.follow(leftMasterDrive);
      // leftDriveFollower2.follow(leftMasterDrive);
      // rightDriveFollower1.follow(rightMasterDrive);
      // rightDriveFollower2.follow(rightMasterDrive);
    }
  }

  /**
   * returns left encoder position
   * 
   * @return left encoder position
   */
  public double getLeftEncoderPosition() {
    return leftControllerDrive.getSelectedSensorPosition(0);
  }

  /**
   * returns right encoder position
   * 
   * @return right encoder position
   */
  public double getRightEncoderPosition() {
    return rightControllerDrive.getSelectedSensorPosition(0);
  }

  public void zeroDriveTrainEncoders() {
    leftControllerDrive.setSelectedSensorPosition(0);
    rightControllerDrive.setSelectedSensorPosition(0);
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public void saveCurrentPose() {
    savedPose = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  /**
   * Returns the heading of the robot in form required for odometry.
   *
   * @return the robot's heading in degrees, from 180 to 180 with positive value
   *         for left turn.
   */
  private double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllerDrive.setVoltage(leftVolts);
    rightControllerDrive.setVoltage(rightVolts);
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    // SmartDashboard.putNumber("Desired L_Velocity", metersPerSecToStepsPerDecisec(leftVelocity));
    // SmartDashboard.putNumber("Desired R_Velocity", metersPerSecToStepsPerDecisec(rightVelocity));
    // SmartDashboard.putNumber("Actual L_Velocity", leftControllerDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Actual R_Velocity", leftControllerDrive.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("L_FeedForward", DriveTrain.FEED_FORWARD.calculate(leftVelocity));
    // SmartDashboard.putNumber("MotorOutput", leftControllerDrive.getMotorOutputVoltage());
    leftControllerDrive.set(TalonFXControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity), DemandType.ArbitraryFeedForward, (DriveTrain.FEED_FORWARD.calculate(leftVelocity))/12);
    rightControllerDrive.set(TalonFXControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity), DemandType.ArbitraryFeedForward, (DriveTrain.FEED_FORWARD.calculate(rightVelocity))/12);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        stepsPerDecisecToMetersPerSec(leftControllerDrive.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightControllerDrive.getSelectedSensorVelocity()));
  }

  public Command createCommandForTrajectory(Trajectory trajectory) {
    InstantCommand initializeDifferentialDrive = new InstantCommand(() -> {
      setUseDifferentialDrive(false); 
    }, this);
    RamseteCommand rc = new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(Auto.RAMSETE_B, Auto.RAMSETE_ZETA),
            DriveTrain.DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this);

    InstantCommand endCommand = new InstantCommand(() -> {
            //setUseDifferentialDrive(true);
            //arcadeDrive(0, 0);
            tankDrive(0, 0);
            IntakeInOutCommand.isDone = true;
            FeederCommand.isDone = true;
        }, this);

    SequentialCommandGroup cg = new SequentialCommandGroup(initializeDifferentialDrive, rc, endCommand);
    return cg;
  }

  /**
   * Converts inches to wheel revolutions
   * 
   * @param inches inches
   * @return wheel revolutions
   */
  public static double insToRevs(double inches) {
    return inches / DriveTrain.WHEEL_CIRCUMFERENCE_INCHES;
  }

  /**
   * Converts inches to encoder steps
   * 
   * @param inches inches
   * @return encoder steps
   */
  public static double insToSteps(double inches) {
    return (insToRevs(inches) * DriveTrain.SENSOR_UNITS_PER_WHEEL_ROTATION);
  }

  /**
   * Converts inches per second to encoder steps per decisecond
   * 
   * @param inchesPerSec inches per second
   * @return encoder steps per decisecond (100 ms)
   */
  public static double insPerSecToStepsPerDecisec(double inchesPerSec) {
    return insToSteps(inchesPerSec) * .1;
  }

  /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (DriveTrain.WHEEL_CIRCUMFERENCE_METERS / DriveTrain.SENSOR_UNITS_PER_WHEEL_ROTATION) * steps;
  }

  public static double metersToSteps(double meters) {
    return ((meters/DriveTrain.WHEEL_CIRCUMFERENCE_METERS) * (DriveTrain.SENSOR_UNITS_PER_WHEEL_ROTATION));
  }

  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return stepsToMeters((int)stepsPerDecisec * 10);
  }

  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec)/10;
  }

  public static final class DriveTrain {
    public static final double SENSOR_UNITS_PER_WHEEL_ROTATION = 2048*14.17;
    public static final double WHEEL_DIAMETER_INCHES = 6.18;//6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 0.626;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double STATIC_VOLTS = .6002;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double VOLT_SECONDS_PER_METER = 3.1246;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0.28332;

    public static final SimpleMotorFeedforward FEED_FORWARD = 
        new SimpleMotorFeedforward(STATIC_VOLTS, VOLT_SECONDS_PER_METER, VOLT_SECONDS_SQUARED_PER_METER);
  }

  public static final class Auto {
    public static final double MAX_VOLTAGE = 12;

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(DriveTrain.FEED_FORWARD, DriveTrain.DRIVE_KINEMATICS, MAX_VOLTAGE);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }


}