/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

import frc.robot.Robot;
import frc.robot.commands.RamseteCommand;

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

  private final DifferentialDrive differentialDrive = new DifferentialDrive(
      new MotorControllerGroup(leftMasterDrive), new MotorControllerGroup(rightMasterDrive));

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
    SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
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
    return leftMasterDrive.getSelectedSensorPosition(0);
  }

  /**
   * returns right encoder position
   * 
   * @return right encoder position
   */
  public double getRightEncoderPosition() {
    return rightMasterDrive.getSelectedSensorPosition(0);
  }

  public void zeroDriveTrainEncoders() {
    leftMasterDrive.setSelectedSensorPosition(0);
    rightMasterDrive.setSelectedSensorPosition(0);
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
    leftMasterDrive.setVoltage(leftVolts);
    rightMasterDrive.setVoltage(rightVolts);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        stepsPerDecisecToMetersPerSec(leftMasterDrive.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightMasterDrive.getSelectedSensorVelocity()));
  }

  public Command createCommandForTrajectory(Trajectory trajectory) {

    InstantCommand initializeDifferentialDrive = new InstantCommand(this, () -> setUseDifferentialDrive(false));
    RamseteCommand rc = new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(Auto.RAMSETE_B, Auto.RAMSETE_ZETA),
            DriveTrain.FEED_FORWARD,
            DriveTrain.DRIVE_KINEMATICS,
            this::getWheelSpeeds,
            new PIDController(DriveTrain.P_GAIN_DRIVE_VEL, 0, DriveTrain.D_GAIN_DRIVE_VEL),
            new PIDController(DriveTrain.P_GAIN_DRIVE_VEL, 0, DriveTrain.D_GAIN_DRIVE_VEL),
            this::tankDriveVolts,
            this);

    InstantCommand endCommand = new InstantCommand(this, () -> {
            //setUseDifferentialDrive(true);
            //arcadeDrive(0, 0);
            tankDrive(0, 0);
        });

    CommandGroup cg = new CommandGroup("TrajectoryDrive");
    cg.addSequential(initializeDifferentialDrive);
    cg.addSequential(rc);;
    cg.addSequential(endCommand);

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
    return (insToRevs(inches) * DriveTrain.SENSOR_UNITS_PER_ROTATION);
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
    return (DriveTrain.WHEEL_CIRCUMFERENCE_METERS / DriveTrain.SENSOR_UNITS_PER_ROTATION) * steps;
  }

  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return stepsToMeters((int)stepsPerDecisec * 10);
  }

  public static final class DriveTrain {
    public static final int DEVICE_ID_LEFT_MASTER = 15;
    public static final int DEVICE_ID_LEFT_SLAVE = 14;
    public static final int DEVICE_ID_LEFT_SLAVE_TWO = 13;
    public static final int DEVICE_ID_RIGHT_MASTER = 0;
    public static final int DEVICE_ID_RIGHT_SLAVE = 1;
    public static final int DEVICE_ID_RIGHT_SLAVE_TWO = 2;

    public static final int SENSOR_UNITS_PER_ROTATION = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 0.626;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double STATIC_VOLTS = 1.2;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double VOLT_SECONDS_PER_METER = 2.79;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0.439;

    public static final SimpleMotorFeedforward FEED_FORWARD = 
        new SimpleMotorFeedforward(STATIC_VOLTS, VOLT_SECONDS_PER_METER, VOLT_SECONDS_SQUARED_PER_METER);

    public static final double P_GAIN_DRIVE_VEL = 2;//0.340d;
    public static final double D_GAIN_DRIVE_VEL = 0;
  }

  public static final class Auto {

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND = Math.sqrt(3);
    public static final double MAX_VOLTAGE = 11;

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(DriveTrain.FEED_FORWARD, DriveTrain.DRIVE_KINEMATICS, MAX_VOLTAGE);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }


}