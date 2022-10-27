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
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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
  private final Field2d m_field = new Field2d();

  private final NavigationBoard gyro = Robot.navigationBoard;
  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;
  private DifferentialDrivetrainSim m_driveSim;
  TalonFXSimCollection m_leftDriveSim = leftControllerDrive.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = rightControllerDrive.getSimCollection();
  PIDController leftController, rightController;
  Timer dtTimer;
  double lastLeftV, lastRightV;

  public DrivetrainTrajectoryExtensions() {
      super();
    dtTimer = new Timer();
    dtTimer.start();
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    leftController = new PIDController(2, 0, 0);
    rightController = new PIDController(2, 0, 0);  
    
    m_driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our identification gains.
      LinearSystemId.identifyDrivetrainSystem(DriveTrain.VOLT_SECONDS_PER_METER, DriveTrain.VOLT_SECONDS_SQUARED_PER_METER, DriveTrain.VOLT_SECONDS_PER_METER, DriveTrain.VOLT_SECONDS_PER_METER),
      DCMotor.getFalcon500(2),       // 2 NEO motors on each side of the drivetrain.
      14.17,                    // 14.17:1 gearing reduction.
      DriveTrain.TRACK_WIDTH_METERS,                  // The track width is 0.7112 meters.
      Units.inchesToMeters(6.18/2), // The robot uses 3" radius wheels.
    
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      null);
     // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
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

    SmartDashboard.putData("Field", m_field);
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
    m_field.setRobotPose(differentialDriveOdometry.getPoseMeters());
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
    SmartDashboard.putNumber("L_MotorOutput", leftVolts);
    SmartDashboard.putNumber("Desired L_Velocity", leftController.getSetpoint());
    SmartDashboard.putNumber("Actual L_Velocity", stepsPerDecisecToMetersPerSec(leftControllerDrive.getSelectedSensorVelocity()));
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    double dt = .02;
    if(dtTimer.get() > 0 && dtTimer.get() < .1)
    {
      dt = dtTimer.get();
    }
    dtTimer.reset();

    SmartDashboard.putNumber("Desired L_Velocity", leftVelocity);
    SmartDashboard.putNumber("Desired R_Velocity", metersPerSecToStepsPerDecisec(rightVelocity));
    SmartDashboard.putNumber("Actual L_Velocity", m_driveSim.getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Actual R_Velocity", rightControllerDrive.getSelectedSensorVelocity());
    double l_ff = DriveTrain.FEED_FORWARD.calculate(lastLeftV, leftVelocity, dt);
    double r_ff = DriveTrain.FEED_FORWARD.calculate(lastRightV, rightVelocity, dt);
    leftControllerDrive.set(TalonFXControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity), DemandType.ArbitraryFeedForward, l_ff/12);
    rightControllerDrive.set(TalonFXControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity), DemandType.ArbitraryFeedForward, r_ff/12);
    SmartDashboard.putNumber("L_FeedForward", l_ff);
    
    SmartDashboard.putNumber("BusVoltage", leftControllerDrive.getBusVoltage());
    lastLeftV = leftVelocity;
    lastRightV = rightVelocity;
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
      differentialDriveOdometry.resetPosition(trajectory.getInitialPose(), Rotation2d.fromDegrees(getHeading()));
      m_field.getObject("traj").setTrajectory(trajectory);
    }, this);
    RamseteController controller = new RamseteController(Auto.RAMSETE_B, Auto.RAMSETE_ZETA);
    controller.setEnabled(false);
    RamseteCommand rc = new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            controller,
            DriveTrain.DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this);
           /* RamseteCommand rc =  new RamseteCommand(
                trajectory,
                this::getCurrentPose,
                controller,
                new SimpleMotorFeedforward(
                    DriveTrain.STATIC_VOLTS,
                    DriveTrain.VOLT_SECONDS_PER_METER,
                    DriveTrain.VOLT_SECONDS_SQUARED_PER_METER),
                DriveTrain.DRIVE_KINEMATICS,
                this::getWheelSpeeds,
                leftController,
                rightController,
                // RamseteCommand passes volts to the callback
                this::tankDriveVolts,
                this);  */      

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
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (steps*DriveTrain.WHEEL_CIRCUMFERENCE_METERS / DriveTrain.SENSOR_UNITS_PER_WHEEL_ROTATION);
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

  @Override 
  public void simulationPeriodic() {
    m_leftDriveSim.setBusVoltage(12);
    m_rightDriveSim.setBusVoltage(12);
    m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                         -m_rightDriveSim.getMotorOutputLeadVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftControllerDrive.getSimCollection().setIntegratedSensorRawPosition((int) metersToSteps(m_driveSim.getLeftPositionMeters()));
    leftControllerDrive.getSimCollection().setIntegratedSensorVelocity((int) metersPerSecToStepsPerDecisec(m_driveSim.getLeftVelocityMetersPerSecond()));
    rightControllerDrive.getSimCollection().setIntegratedSensorRawPosition(-(int) metersToSteps(m_driveSim.getRightPositionMeters()));
    rightControllerDrive.getSimCollection().setIntegratedSensorVelocity(-(int) metersPerSecToStepsPerDecisec(m_driveSim.getRightVelocityMetersPerSecond()));    
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveSim.getCurrentDrawAmps()));
    SmartDashboard.putNumber("CalculatedBusVoltage", BatterySim.calculateDefaultBatteryLoadedVoltage(m_driveSim.getCurrentDrawAmps()));
  }

  public static final class DriveTrain {
    public static final double SENSOR_UNITS_PER_WHEEL_ROTATION = 2048*14.17;
    public static final double WHEEL_DIAMETER_INCHES = 6.18;//6d;
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

    public static TrajectoryConfig TRAJ_CONFIG = new TrajectoryConfig(2.5, 2.5).addConstraint(VOLTAGE_CONSTRAINT);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }


}