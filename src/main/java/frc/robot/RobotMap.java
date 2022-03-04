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
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
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
    public static double SHOOTER_SPEED_TOLERANCE = 200;
    public static double SHOOTER_DEFAULT_HIGH_HUB = 2900;
    public static double SHOOTER_DEFAULT_LOW_HUB = 1500;
    public static double SHOOTER_ks = 0.31651;
    public static double SHOOTER_kv = 0.0030083*42;
  }

  public static class IntakeDevices {
    public static int INTAKE_CONTROLLER_ID = 8;
    public static VictorSPX intakeVictor = new VictorSPX(INTAKE_CONTROLLER_ID);
    // TODO: change to real number
    public static int FORWARD_CHANNEL = 6;
    public static int REVERSE_CHANNEL = 1;
    public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL,
        REVERSE_CHANNEL);
    public static double intakeSpeed = 1;
    public static double outtakeSpeed = 0.50;

  }

  // MecanumDevices is a secondary set of wheels after the intake
  public static class MecanumDevices {
    public static int MECANUM_CONTROLLER_ID = 9;
    public static VictorSPX mecanumVictor = new VictorSPX(MECANUM_CONTROLLER_ID);
    public static double mecanumInSpeed = 0.75;
    public static double mecanumOutSpeed = 0.50;
  }

  public static class FeederDevices {
    public static int FEEDER_CONTROLLER = 10;
    public static CANSparkMax feederController = new CANSparkMax(FEEDER_CONTROLLER, MotorType.kBrushless);
     public static ColorSensorV3 ballColor = new ColorSensorV3(I2C.Port.kOnboard);// make this work on kMXP or robot

    public static enum FeederDirection {
      up,
      down
    }
  }

  public static class HangerDevices {
    public static int HANGER_CONTROLLER = 2;
    public static WPI_TalonFX hangerTalon = new WPI_TalonFX(HANGER_CONTROLLER);
    public static double controllerDeadzone = 0.2;
    //TODO: these are placeholder values, change them when we know more about the needed speeds
    public static double hangerUpSpeed = 0.50;
    public static double hangerDownSpeed = -0.75;
    public static double upSoftLimitThreshold = 0;
    public static double downSoftLimitThreshold = 0;
  }
}
