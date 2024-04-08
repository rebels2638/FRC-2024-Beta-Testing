
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants;
import frc.robot.commands.autoAligment.LocalADStarAK;
import frc.robot.commands.compositions.CancelIntakeNote;
import frc.robot.lib.swervelib.SwerveDrive;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  SendableChooser<Double> gyroOffset = new SendableChooser<>();



  private Timer time;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  String logPath = "/Users/conne/Downloads/"; // TODO: Remember to change this value guys (Edan)
  @Override
  public void robotInit() { 
    SignalLogger.enableAutoLogging(false); // TODO: ABSOLUTELY NEED TO BE HERE FOR COMPS
    Pathfinding.setPathfinder(new LocalADStarAK());

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

  // Set up data receivers & replay source
    switch (Constants.currentMode) {
      // Running on a real robot, log to a USB stick
      case REAL:
        // Logger.addDataReceiver(new WPILOGWriter("D:/"));
        // Logger.addDataReceiver(new NT4Publisher());
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        // Logger.addDataReceiver(new WPILOGWriter(logPath));
        // Logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        // Logger.setReplaySource(new WPILOGReader(logPath));
        // // Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter(logPath));
        break;
    }
    // SwerveSubsystem.getInstance().setGyro(180);
    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    // Instantiate our RobotContainer.  This will perform all 
    // CameraServer.startAutomaticCapture();
    m_robotContainer = RobotContainer.getInstance();
    // SwerveSubsystem.getInstance().setGyro(180);

    time = new Timer();
    CommandScheduler.getInstance().enable();
    gyroOffset.setDefaultOption("NoOffset", 0.0);

    gyroOffset.addOption("BlueMidSpeaker", 180.0);
    gyroOffset.addOption("RedMidSpeaker" , 180.0);
    gyroOffset.addOption("FacingOppSide(Battery is)", 0.0);
    gyroOffset.addOption("AmpSide", -240.0);
    gyroOffset.addOption("SourceSide", 120.0);
    
    Shuffleboard.getTab("auto").add("Angle Offset", gyroOffset);

  }

   /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
   @Override
   public void autonomousInit() {

    // LEDSubsystem.getInstance().setColor(0.87);
     time.reset();
     time.start();
     
    //  Pose2d currentPose = SwerveSubsystem.getInstance().getPose();
    //  SwerveSubsystem.getInstance().resetOdometry(currentPose);
     
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();
     // schedule the autonomous command (example)
     if (m_autonomousCommand != null) {
      
      m_autonomousCommand.schedule();
     }
   }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

  }


  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LEDSubsystem.getInstance().setColor(0.77);
    //Reflect the angle for red only
    // var alliance = DriverStation.getAlliance(); 
    // if(alliance.get() == DriverStation.Alliance.Red){
    //   yaw = yaw.plus(new Rotation2d(Math.PI));
    // }
    // SwerveSubsystem.getInstance().zeroGyro();
    //you might be wondering why it is here and not on resetOdometryAuto, this is here only because I need to it apply once, pathplanner calls resetOdometryAuto(the supplied command) multiple times possibly after every path end from my tests
    SwerveSubsystem.getInstance().resetOdometry(new Pose2d(new Translation2d(0,0), SwerveSubsystem.getInstance().getYaw().rotateBy(Rotation2d.fromDegrees(gyroOffset.getSelected()))));

    //UNCOMMENT THE LINE BELOW AND COMMENT THE LINE ABOVE IF YOU ARE GETTING FPGA::WPI.now() ERROR
    // SwerveSubsystem.getInstance().resetOdometry(new Pose2d(new Translation2d(0,0), SwerveSubsystem.getInstance().getYaw().rotateBy(Rotation2d.fromDegrees(0))));

    // SwerveSubsystem.getInstance().setGyro(SwerveSubsystem.getInstance().getYaw().rotateBy(new Rotation2d(Math.toRadians(gyroOffset.getSelected()))).getDegrees()); //TODO: DONT USE THIS FOR NOW
    

    // reset the intake at the start of teleop 
    // new CancelIntakeNote().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotContainer.checkControllers();
    //System.out.println(logPath);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}
