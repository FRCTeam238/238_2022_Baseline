/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
import frc.robot.commands.TankDrive;
import frc.robot.commands.drivetrainparameters.DriverJoysticks;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {

  // private LiveWindow lw = LiveWindow.getInstance();

  public final static double TICKS_PER_INCH = 193;
  private final static double ANGLE_KP = 3;
  private final double kV = 3.47;// 0.00434
  private final double kA = 0.225;// 00.00434 * 0.15;
  private final double vSetpoint = 1.078;// 0.078

  protected final static WPI_TalonFX rightControllerDrive = RobotMap.DrivetrainControllers.RightDriveTrainController;
  protected final static WPI_TalonFX leftControllerDrive = RobotMap.DrivetrainControllers.LeftDriveTrainController;

  protected final WPI_TalonFX leftDriveFollower1 = RobotMap.DrivetrainControllers.LeftDriveTrainFollower;
  protected final WPI_TalonFX rightDriveFollower1 = RobotMap.DrivetrainControllers.RightDriveTrainFollower;

  

  public static double kP = 1;
  public static double kI = 0;
  public static double kD = 0;
  public static double kF = 1023 / 6800;
  public static int kIzone = 400;
  public static int kPIDLoopIdx = 0;
  public static int kTimeoutMs = 30;
  public static double rampRate = 0.25;

  private double diagnosticStartTime = 0;

  private NetworkTableEntry entryLeft;

  private NetworkTableEntry entryRight;

  Dashboard238 dashboard;

  public Drivetrain() {
    initTalons();
    // initLiveWindow();
    dashboard = Robot.dashboard238;
    entryLeft = Shuffleboard.getTab("DiagnosticTab").add("LeftPower", 0).getEntry();
    entryRight = Shuffleboard.getTab("DiagnosticTab").add("RightPower", 0).getEntry();
    // rightControllerDrive.setInverted(InvertType.None);
  }

  @Override
  public void initDefaultCommand() {
    DriverJoysticks myDriverJoysticks = new DriverJoysticks();
    myDriverJoysticks.invertJoysticks();
    TankDrive tankDriveCommand = new TankDrive(myDriverJoysticks, this);
    setDefaultCommand(tankDriveCommand);
    SmartDashboard.putData("Drivetrain command", this);
  }

  public void drive(double left, double right) {
    leftControllerDrive.set(ControlMode.PercentOutput, left);
    rightControllerDrive.set(ControlMode.PercentOutput, right);
  }

  public void drive(double left, double right, double desiredAngle) {

    if (desiredAngle == 0) {
      drive(left, right);
    } else {
      if (Math.abs(desiredAngle) > (360.0 - 0.0) / 2.0D) {
        desiredAngle = desiredAngle > 0.0D ? desiredAngle - 360.0 + 0.0 : desiredAngle + 360.0 - 0.0;
      }

      double angleVelocityAddend = desiredAngle * ANGLE_KP;
      angleVelocityAddend = Math.min(50, Math.max(angleVelocityAddend, -50));

      accelerate(left + angleVelocityAddend, right - angleVelocityAddend, left, right);
    }
  }

  private static double calcTicks(double distance){
    return distance * TICKS_PER_INCH;
  }

  /** Drive a number of inches using TalonSRX PID loops */
  public void driveWithTicks(double distance) {
    initPID(rightControllerDrive);
    initPID(leftControllerDrive);
    //resetEncoders() is done in initPID
    double ticks = calcTicks(distance);
    setPosition(rightControllerDrive, ticks);
    setPosition(leftControllerDrive, ticks);
  }

  // method to accelerate rather than set straigt power
  public void accelerate(double leftSpeed, double rightSpeed, double leftAccel, double rightAccel) {

    /*
     * the joystick value is multiplied by a target RPM so the robot works with the
     * velocity tuning code
     */

    double leftWantedVoltage = 0;
    leftWantedVoltage += kV * leftSpeed;
    leftWantedVoltage += kA * leftAccel;

    leftWantedVoltage += leftWantedVoltage > 0 ? vSetpoint : -vSetpoint;

    double rightWantedVoltage = 0;
    rightWantedVoltage += kV * rightSpeed;
    rightWantedVoltage += kA * rightAccel;
    rightWantedVoltage += rightWantedVoltage > 0 ? vSetpoint : -vSetpoint;

    leftControllerDrive.set(ControlMode.Velocity, (-leftSpeed) * TICKS_PER_INCH / 10.0);
    rightControllerDrive.set(ControlMode.Velocity, (-rightSpeed) * TICKS_PER_INCH / 10.0);
    // Logger.Log("DriveTrain.driveSpeedAccel() LEFT Speed = " + -leftSpeed + "RIGHT
    // Speed = " + -rightSpeed);
    // Logger.Log("Drive Accel RIGHT Speed:" + -rightSpeed);
    // convert to inches/second
    // Logger.Log("DriveTrain() : driveSpeed() : RIGHT SPEED IS ="
    // + leftFrontDrive.getSelectedSensorVelocity(0) /
    // CrusaderCommon.DRIVE_FORWARD_ENCODER_TICKS_PER_INCH);
    // Logger.Log("DriveTrain() : driveSpeed() : RIGHT ERROR IS =" +
    // rightFrontDrive.getClosedLoopError(0));

  }

  public void stop() {
    leftControllerDrive.set(ControlMode.PercentOutput, 0);
    rightControllerDrive.set(ControlMode.PercentOutput, 0);
  }

  public void initTalons() {

    //TalonSRX_238 factory method returns preconfigured talks with defaults, no need to reset to defaults

    leftDriveFollower1.follow(leftControllerDrive);
  
    rightDriveFollower1.follow(rightControllerDrive);
  
    leftControllerDrive.setNeutralMode(NeutralMode.Brake);
    leftDriveFollower1.setNeutralMode(NeutralMode.Brake);
  
    rightControllerDrive.setInverted(true);
    rightDriveFollower1.setInverted(true);
  
    rightControllerDrive.setNeutralMode(NeutralMode.Brake);
    rightDriveFollower1.setNeutralMode(NeutralMode.Brake);
  
    rightControllerDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);
    leftControllerDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 0);

    initPID(rightControllerDrive);
    initPID(leftControllerDrive);

    Logger.Debug("initTalons Is Sucessful!");
  }

  public void initPID(TalonFX talon) {

   // TalonSRX_238.initPID(leftMasterDrive, kP, kI, kD, kF, kIzone, kTimeoutMs, kPIDLoopIdx, rampRate);
    //TalonSRX_238.initPID(rightMasterDrive, kP, kI, kD, kF, kIzone, kTimeoutMs, kPIDLoopIdx, rampRate);
    //public static void initPID(TalonSRX talon, double kP, double kI, double kD, double kF, int kIzone, int kTimeoutMs, int kPIDLoopIdx, double rampRate){

      /* Config the peak and nominal outputs ([-1, 1] represents [-100, 100]%) */
      talon.configNominalOutputForward(0, kTimeoutMs);
      talon.configNominalOutputReverse(0, kTimeoutMs);
      talon.configPeakOutputForward(1, kTimeoutMs);
      talon.configPeakOutputReverse(-1, kTimeoutMs);
  
      /**
       * Config the allowable closed-loop error, Closed-Loop output will be
       * neutral within this range.
       */
      talon.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);
  
      /* Config closed loop gains for Primary closed loop (Current) */
      talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
      talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
      talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
      talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
      talon.config_IntegralZone(kPIDLoopIdx, kIzone, kTimeoutMs);
      
      // Config encoders
      talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
  
      talon.configClosedloopRamp(rampRate, kTimeoutMs);
  
      // Ensure motor output and encoder velocity are proportional to each other
      // If they become inverted, set these to true
      talon.setSensorPhase(true);
  
      resetEncoder(talon, kPIDLoopIdx, kTimeoutMs);
    }

    public void resetEncoder(TalonFX talon, int kPIDLoopIdx, int kTimeoutMs){
      talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }
    public void resetEncoders(){
      resetEncoder(rightControllerDrive, kPIDLoopIdx, kTimeoutMs);
      resetEncoder(leftControllerDrive, kPIDLoopIdx, kTimeoutMs);
      
    }


  

  //public void resetEncoders() {
   // TalonSRX_238.zeroEncoder(rightMasterDrive, kPIDLoopIdx, kTimeoutMs);
   /// TalonSRX_238.zeroEncoder(leftMasterDrive, kPIDLoopIdx, kTimeoutMs);
  //}

  public double getLeftEncoderTicks() {
    double leftTicks = getTicks(leftControllerDrive);
    Logger.Trace("LEFT TICKS: " + leftTicks);
    return leftTicks;
  }

  // return distance travelled in inches
  public double leftDistanceTravelled() {
    double leftTicks = getLeftEncoderTicks();
    double leftDistanceTravelled = leftTicks / TICKS_PER_INCH;
    Logger.Trace("LEFT TICKS: " + leftTicks + "  Left Distance Travelled  " + leftDistanceTravelled);
    return leftDistanceTravelled;
  }

  public double getRightEncoderTicks() {
    double rightTicks = getTicks(rightControllerDrive);
    Logger.Trace("RIGHT TICKS: " + rightTicks);
    return rightTicks;
  }

  // return distance travelled in inches
  public double rightDistanceTravelled() {
    double rightTicks = getRightEncoderTicks();
    double rightDistanceTravelled = rightTicks / TICKS_PER_INCH;
    Logger.Trace("RIGHT TICKS: " + rightTicks);
    return rightDistanceTravelled;
  }

  public double getLeftPower(){
    double power = leftControllerDrive.getMotorOutputPercent();
    return power;
  }

  public double getRightPower(){
    double power = rightControllerDrive.getMotorOutputPercent();
    return power;
  }

  public boolean isAtPosition(double distance) {
    double desiredTicks = calcTicks(distance);
    double rightPosition = getTicks(rightControllerDrive);
    double leftPosition = getTicks(leftControllerDrive);
    double averagePosition = (rightPosition + leftPosition) / 2;
    Logger.Debug("Current position: " + averagePosition);
    Logger.Debug("Desired position: " + desiredTicks);
    boolean isDone = (Math.abs(averagePosition) >= Math.abs(desiredTicks));
    return isDone;
  }

  private static double getTicks(TalonFX talon){
    return talon.getSelectedSensorPosition();
  }

  private static void setPosition(TalonFX talon, double ticks) {
    talon.set(ControlMode.Position, ticks);
  }

  private void initLiveWindow() {
    SendableWrapper leftEncoder = new SendableWrapper(builder -> {
      builder.addDoubleProperty("Ticks", this::getLeftEncoderTicks, null);
    });

    SendableWrapper rightEncoder = new SendableWrapper(builder -> {
      builder.addDoubleProperty("Ticks", this::getRightEncoderTicks, null);
    });

    SendableWrapper leftSpeedController = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("Speed Controller");
      builder.addDoubleProperty("Value", () -> leftControllerDrive.getMotorOutputPercent(), null);
    });

    SendableWrapper rightSpeedController = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("Speed Controller");
      builder.addDoubleProperty("Value", () -> rightControllerDrive.getMotorOutputPercent(), null);
    });

    SendableWrapper pidSendable = new SendableWrapper(builder -> {
      builder.setSmartDashboardType("PIDController");
      builder.addDoubleProperty("p", () -> kP, (value)-> kP = value);
      builder.addDoubleProperty("i", () -> kI, (value)-> kI = value);
      builder.addDoubleProperty("d", () -> kD, (value)-> kD = value);
      builder.addBooleanProperty("enabled", () -> rightControllerDrive.getControlMode() == ControlMode.Position, null);
      builder.addDoubleProperty("setpoint", () -> 0.0 , null);
      builder.addDoubleProperty("f", () -> kD, (value)-> kD = value);

    });

    addChild("PIDController", pidSendable);
    addChild("Left Encoder", leftEncoder);
    addChild("Right Encoder", rightEncoder);
    addChild("Left Speed Controller", leftSpeedController);
    addChild("Right Speed Controller", rightSpeedController);
  }

  private List<SendableWrapper> _sendables = new ArrayList<>();
  private void addChild(String name, SendableWrapper wrapper){
    _sendables.add(wrapper);
    addChild(name, (Sendable)wrapper);
  }

  public void runDrivetrainDiagnostics(){
    Shuffleboard.selectTab("DiagnosticTab");
    if(diagnosticStartTime == 0){
        diagnosticStartTime = Timer.getFPGATimestamp();
    }
    if((diagnosticStartTime + 3) <= Timer.getFPGATimestamp() && diagnosticStartTime != 0){
       stop();
    }else{
      drive(0.5, 0.5);
      double leftPower = getLeftPower();
      double rightPower = getRightPower();
      entryLeft.setDouble(leftPower);
      entryRight.setDouble(rightPower);
    }

  }

}
