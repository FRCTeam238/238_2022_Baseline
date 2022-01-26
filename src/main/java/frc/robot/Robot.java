/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.core238.autonomous.AutonomousModesReader;
import frc.core238.autonomous.DataFileAutonomousModeDataSource;
import frc.core238.autonomous.IAutonomousModeDataSource;
import frc.robot.commands.DriveStraightNavBoard;

import frc.robot.subsystems.DrivetrainTrajectoryExtensions;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.NavigationBoard;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static DrivetrainTrajectoryExtensions drivetrain;
  public static NavigationBoard navigationBoard;
  public static Vision vision;
  public static OI oi;
  public static Dashboard238 dashboard238;
  public static Shooter shooter;
  public static Feeder feeder;
  public static Hanger hanger;
  public static Intake intake;
  public static LED led;

  // Dictionary of auto mode names and commands to run
  HashMap<String, CommandGroup> m_autoModes;
  CommandGroup m_autoCommandGroup;

  // this is set to false once we've alreay hit auto once or gone in to teleop,
  // prevents us from running auto twice
  boolean m_allowAuto = true;

  SendableChooser<String> m_chooser = new SendableChooser<>();

  boolean fmsConnected = false;
  boolean enteredTeleop = false;

  public Robot() {
    navigationBoard = new NavigationBoard();
    drivetrain = new DrivetrainTrajectoryExtensions();
    vision = new Vision(FieldConstants.VisionConstants.targetHeight, FieldConstants.VisionConstants.cameraHeight);
    shooter = new Shooter();
    dashboard238 = new Dashboard238();
    feeder = new Feeder();
    //hanger = new Hanger();
    intake = new Intake();
    led = new LED(2, 150);

    //SmartDashboard.putData(TrajectoryDrive.getExampleCommand());
  }
  
  @Override
  public void robotInit() {
    oi = new OI();
    //navigationBoard.init();
    dashboard238.init();
    vision.initLimelight();
    // panelManipulator.initSensor();
    populateAutomodes();
    // List<String> params= new ArrayList<>();
    // params.add("Straight");
    // TrajectoryDriveCommand driveStraightTrajectory = new TrajectoryDriveCommand();
    // driveStraightTrajectory.setParameters(params);
    // SmartDashboard.putData("Drive Straight - trajectory", driveStraightTrajectory);
    // LiveWindow.disableAllTelemetry();

    SmartDashboard.putNumber("Shooter Speed Scalar", 1);
  }

  private void populateAutomodes() {

    if (isReal()){
    // initialize the automodes list
      IAutonomousModeDataSource autoModesDataSource = new DataFileAutonomousModeDataSource("/home/lvuser/deploy/amode238.txt");
      AutonomousModesReader reader = new AutonomousModesReader(autoModesDataSource);
      m_autoModes = reader.getAutonmousModes();
    } else {
      m_autoModes = new HashMap<>();

      // CommandGroup cg = new CommandGroup();
      // // DriveStraightNavBoard cmd = new DriveStraightNavBoard();
      // DriveStraightPID drivefrwrd = new DriveStraightPID(48);
      // DriveStraightPID drivebckwrd = new DriveStraightPID(-72);
      // //List<String> parameters = new ArrayList<>();
      // //parameters.add("5");
      // //parameters.add("10");
      // //cmd.setParameters(parameters);
      // cg.addSequential(drivefrwrd);
      // cg.addSequential(drivebckwrd);

      // m_autoModes.put("Drive forward, then backward", cg);

      CommandGroup cg2 = new CommandGroup();
      DriveStraightNavBoard cmd2 = new DriveStraightNavBoard();
      List<String> parameters2 = new ArrayList<>();
      parameters2.add("15");
      parameters2.add("100");
      cmd2.setParameters(parameters2);
      cg2.addSequential(cmd2);

      DriveStraightNavBoard cmd3 = new DriveStraightNavBoard();
      List<String> parameters3 = new ArrayList<>();
      parameters3.add("5");
      parameters3.add("1000");
      cmd3.setParameters(parameters3);
      cg2.addSequential(cmd3);

      //m_autoModes.put("Simulate - Drive Straight: sp 15, dist 100", cg2);
      // CommandGroup cg3 = new CommandGroup();
      // cg3.addSequential(cg);
      // cg3.addSequential(cg2);
      // m_autoModes.put("Simulate - Auto1/Auto2", cg3);
    }

    if (m_autoModes.size() == 0){
      CommandGroup cg = new CommandGroup();
      DriveStraightNavBoard cmd = new DriveStraightNavBoard();
      List<String> parameters = new ArrayList<>();
      parameters.add("0.5");
      parameters.add("1000000");
      cmd.setParameters(parameters);
      cg.addSequential(cmd);
      m_autoModes.put("No auto modes found -- default command!", cg);
    }

    if (m_autoModes.size() > 0) {
      Boolean first = true;

      for (var entry : m_autoModes.entrySet()) {
        String name = entry.getKey();
        if (first) {
          first = false;
          m_chooser.setDefaultOption(name, name);
        } else {
          m_chooser.addOption(name, name);
        }
      }
      SmartDashboard.putData("Auto Modes", m_chooser);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Can Run Auto", m_allowAuto);
    feeder.countHeldBalls();
    SmartDashboard.putNumber("Shooter RPM", shooter.getSpeed());
    SmartDashboard.putNumber("Turret X error", vision.getYaw());

    shooter.increaseSpeed(SmartDashboard.getNumber("Shooter Speed Scalar", 1));
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    if(enteredTeleop){
      Shuffleboard.stopRecording();
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    String autoMode = m_chooser.getSelected();
    // make sure we have a commandgroup corresponding to the autmode
    // AND it's ok to run auto. m_allowAuto will be false if teleop ini as been run
    // or we've run this init at least once
    if (m_allowAuto && m_autoModes.containsKey(autoMode)) {
      m_autoCommandGroup = m_autoModes.get(autoMode);
      m_autoCommandGroup.start();
    }

    // prevent the robot from rerunning auto mode a second time without a restart
    m_allowAuto = false;
    if(DriverStation.isFMSAttached()){
      Shuffleboard.startRecording();
      fmsConnected = true;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // prevent auto from running after teleop has been initialized
    m_allowAuto = false;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommandGroup != null) {
      m_autoCommandGroup.cancel();
    }

    if(fmsConnected){
      enteredTeleop = true;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    String getDataFromDriverStation = DriverStation.getGameSpecificMessage();
    //SmartDashboard.putString("Assigned Color", getDataFromDriverStation);
    vision.postValues();


    

    //SmartDashboard.putData("RUN SHOOTER TEST", new SetShooterSpeedCommand(SmartDashboard.getNumber("Shooter RPM", 0)));
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testPeriodic() {

    // vision.trackingMode();
    // vision.ledsOn();
    // navigationBoard.navxValues();
    vision.ledsOn();
    vision.trackingMode();
    vision.postValues();

    shooter.runShooterDiagnostics();
    //feeder.runFeederDiagnostics();
    //intake.runIntakeDiagnostics();
    //navigationBoard.runNavBoardDiagnostics();
    //drivetrain.runDrivetrainDiagnostics();
  }
}
