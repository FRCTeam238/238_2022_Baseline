/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Used to map hardware ports and define hardware objects - motor controllers,
 * joysticks, cameras, etc
 */
public final class RobotMap {

  /** Motor controllers for the drive train */
  public static class DrivetrainControllers {
    public static final int DRIVE_TRAIN_RIGHT_CONTROLLER = 14;
    public static final int DRIVE_TRAIN_RIGHT_FOLLOWER = 15;

    public static final int DRIVE_TRAIN_LEFT_CONTROLLER = 0;
    public static final int DRIVE_TRAIN_LEFT_FOLLOWER = 1;

    public static WPI_TalonFX LeftDriveTrainController = new WPI_TalonFX(DRIVE_TRAIN_LEFT_CONTROLLER);
    public static WPI_TalonFX LeftDriveTrainFollower = new WPI_TalonFX(DRIVE_TRAIN_LEFT_FOLLOWER);

    public static WPI_TalonFX RightDriveTrainController = new WPI_TalonFX(DRIVE_TRAIN_RIGHT_CONTROLLER);
    public static WPI_TalonFX RightDriveTrainFollower = new WPI_TalonFX(DRIVE_TRAIN_RIGHT_FOLLOWER);
  }

  /** Driver joysticks and operator controllers */
  public static class Joysticks {
    public static int leftStickPort = 2;
    public static int rightStickPort = 1;
    public static int controllerPort = 0;

    public static Joystick driverStickLeft = new Joystick(leftStickPort);
    public static Joystick driverStickRight = new Joystick(rightStickPort);
    public static XboxController operatorController = new XboxController(controllerPort);
  }

  /**
   * Integer settings for the vision subsystem - change camera mode, pipeline, and
   * LED state using these
   */
  public static class LimelightSettings {
    public static int visionMode = 0;
    public static int cameraMode = 1;
    public static int ledsOn = 3;
    public static int ledsBlink = 2;
    public static int ledsOff = 1;
  }

  public static class ShooterDevices {
    public static int SHOOTER_CONTROLLER = 13;
    public static int SHOOTER_FOLLOWER = 12;
    public static CANSparkMax shooterController = new CANSparkMax(SHOOTER_CONTROLLER, MotorType.kBrushless);
    public static CANSparkMax shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);

  }

  public static class IntakeDevices {
    public static int INTAKE_CONTROLLER_ID = 8;
    public static VictorSPX intakeVictor = new VictorSPX(INTAKE_CONTROLLER_ID);
    // TODO: change to real number
    public static int FORWARD_CHANNEL = 1;
    public static int REVERSE_CHANNEL = 6;
    public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL, REVERSE_CHANNEL);

  }
// MecanumDevices is a secondary set of wheels after the intake
  public static class MecanumDevices {
    public static int MECANUM_CONTROLLER_ID = 9;
    public static VictorSPX mecanumVictor = new VictorSPX(MECANUM_CONTROLLER_ID);
  }

  public static class FeederDevices {
    public static int FEEDER_CONTROLLER = 10;
    public static CANSparkMax feederController = new CANSparkMax(FEEDER_CONTROLLER, MotorType.kBrushless);
    // public static VictorSPX transportVictor = new VictorSPX(TRANSPORT_CONTROLLER);
  }

  public static class HangerDevices {
    public static int HANGER_CONTROLLER = 2;
    public static WPI_TalonFX hangerTalon = new WPI_TalonFX(HANGER_CONTROLLER);
   

  }
}
