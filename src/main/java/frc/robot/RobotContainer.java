// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;

import java.io.File;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.drivetrain.TurnCommand;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorToggle;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.commands.shooter.ShooterWindReverse;
import frc.robot.commands.compositions.Thing6;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystemIO;
import frc.robot.subsystems.swerve.SwerveSubsystemIORunning;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.OperatorConstants;
// import frc.robot.commands.automation.AutoAlign;
// import frc.robot.commands.drivetrain.AbsoluteDrive;
// import frc.robot.commands.pivot.RollIntake;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.Intake.RollIntakeEject;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberRaw;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.commands.compositions.CancelIntakeNote;
import frc.robot.commands.compositions.Climb;
import frc.robot.commands.compositions.FeedAndHoldNote;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.IntakeNoteAuto;
import frc.robot.commands.compositions.ScoreAMP;
import frc.robot.commands.compositions.ShootNote;
import frc.robot.commands.compositions.ShootNoteAuto;
import frc.robot.commands.compositions.ShootNoteTele;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.audio.AudioPlayer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOFalcon;
// import frc.robot.subsystems.elevator.ElevatorIOSim;;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.poseLimelight.PoseLimelight;
import frc.robot.subsystems.poseLimelight.PoseLimelightIO;
import frc.robot.subsystems.poseLimelight.PoseLimelightIOReal;
import frc.robot.subsystems.poseLimelight.PoseLimelightIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIONeo;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIONeo;
import frc.robot.subsystems.shooter.ShooterIOSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.motorcontrol.Spark;



// import frc.robot.commands.drivetrain.TeleopDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer instance = null;
  public Spark color1 = new Spark(6);
  public Spark color2 = new Spark(7);

  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;
  private final XboxController xboxTester;
  
  // Robot Subsystems
  private SwerveSubsystem swerveSubsystem;
  private final AbsoluteFieldDrive closedFieldAbsoluteDrive;
  
  private final Intake intakeSubsystem;
  private final Shooter shooterSubsystem;
  private final PoseLimelight visionSubsystem;
  private final AutoRunner autoRunner;

  private final Elevator elevatorSubsystem;
  private final Pivot pivotSubsystem;
  private final Climber climberSubsystem;

  // private HttpCamera limelightFeed;

  public RobotContainer() {
  // limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
  // Shuffleboard.getTab("DriverStation").add("LL", limelightFeed).withPosition(0, 0).withSize(15, 8).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

     
    // UsbCamera Camera = CameraServer.startAutomaticCapture();
    
    // setInstances();
    // Instantiate our controllers with proper ports.
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);
    
    // not sim'ed or replayed
    // aPlayer = new AudioPlayer();
    // poseLimelightSubsystem = new PoseLimelight(new PoseLimelightIOReal() {});

    switch (Constants.currentMode) {
      case SIM:
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIOSim())); //Assigns the instance object(pointer) to the variable so no new changes are needed.
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIOSim()));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIOSim()));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIOSim())); 
        climberSubsystem = Climber.setInstance(new Climber(new ClimberIOSim()));

        visionSubsystem = new PoseLimelight(new PoseLimelightIOSim());
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon") /*, visionSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));

        break; // 2.43, 4.13
      
      case REPLAY:
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIO(){}));
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIO(){}));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIO(){}));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIO(){}));  
        
        climberSubsystem = Climber.setInstance(new Climber(new ClimberIO(){}));

        visionSubsystem = new PoseLimelight(new PoseLimelightIO() {});
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon")/* ,visionSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIO() {});

        break;
        
      default:
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIONeo()));
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIONeo()));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIOFalcon()));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIONeo()));
        climberSubsystem = Climber.setInstance(new Climber(new ClimberIOFalcon()));

        visionSubsystem = new PoseLimelight(new PoseLimelightIOReal());
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon")/* , visionSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));
        swerveSubsystem = SwerveSubsystem.setInstance(swerveSubsystem);

        break;
    }
    
    autoRunner = new AutoRunner(swerveSubsystem);

    SequentialCommandGroup intake, feedHold;


    closedFieldAbsoluteDrive = new AbsoluteFieldDrive(swerveSubsystem,
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), false);

    NamedCommands.registerCommand("MoveElevatorAMP", new MoveElevatorAMP());
    NamedCommands.registerCommand("MoveElevatorTurtle", new MoveElevatorTurtle());
    NamedCommands.registerCommand("ShooterWindUp", new ShooterWindup());
    NamedCommands.registerCommand("RollIntakeIn", new RollIntakeIn());
    NamedCommands.registerCommand("StopIntake", new StopIntake());
    NamedCommands.registerCommand("IntakeNote", intake = new IntakeNoteAuto());
    NamedCommands.registerCommand("ShooterStop", new ShooterStop());
    NamedCommands.registerCommand("ShooterWindReverse", new ShooterWindReverse());
    NamedCommands.registerCommand("ShootNote", new ShootNote());
    NamedCommands.registerCommand("ShootNoteAuto", new ShootNoteAuto());
    NamedCommands.registerCommand("PivotToTorus", new PivotToTorus());
    NamedCommands.registerCommand("CancelIntakeNote", new CancelIntakeNote(intake, null));
    NamedCommands.registerCommand("RollIntakeIn", new RollIntakeIn());


    // swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);
    // climberSubsystem.setDefaultCommand(new MoveClimberRaw(climberSubsystem, xboxTester));
    // xboxTester.getAButton().onTrue(new PivotToTorus());
    // xboxTester.getBButton().onTrue(new MoveElevatorAMP());
    // xboxTester.getYButton().onTrue(new MoveElevatorTurtle());

    // // xboxTester.getXButton().onTrue(new PivotTurtle());
    // xboxTester.getXButton().onTrue(new MoveClimberDown());
    // xboxTester.getAButton().whileTrue(new TurnCommand(swerveSubsystem));
    // xboxTester.getLeftMiddleButton().onTrue(new Climb());
    // // xboxTester.getRightMiddleButton().onTrue(new Thing6());
    xboxTester.getLeftBumper().onTrue(new InstantCommand(()-> climberSubsystem.zeroHeight()));
    // xboxTester.getRightBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.zeroHeight()));

    this.xboxDriver.getBButton().onTrue(new MoveClimberUp());
    this.xboxDriver.getAButton().onTrue(new MoveClimberDown()); // TODO: recomment

    
    // //Michaelangelo controls
    this.xboxOperator.getLeftBumper().onTrue(new ShooterStop());
    this.xboxOperator.getRightBumper().onTrue(new ShooterWindup());
    this.xboxOperator.getXButton().onTrue(new MoveElevatorToggle());
    this.xboxOperator.getYButton().onTrue(new ScoreAMP()); // changed
    this.xboxOperator.getAButton().onTrue(new RollIntakeIn()); // change back to shootNoteTele
    this.xboxOperator.getBButton().onTrue(feedHold = new FeedAndHoldNote());
    this.xboxOperator.getRightMiddleButton().onTrue(new RollIntakeEject());
        //TrevorBallshack Controls
    // swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);
    this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    this.xboxDriver.getLeftBumper().onTrue(intake = new IntakeNote());
    // this.xboxDriver.getLeftBumper().onTrue(intake = new IntakeNoteAuto());
    this.xboxDriver.getRightMiddleButton().onTrue(new RollIntakeEject());
    this.xboxDriver.getRightBumper().onTrue(new CancelIntakeNote(intake, feedHold));
    // this.xboxDriver.getYButton().onTrue(new Climb());
    // Shuffleboard.getTab("Auto").add("Zero Swerve", new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    // this.xboxDriver.getAButton().onTrue(new PivotTurtle());
    // this.xboxDriver.getLeftMiddleButton().onTrue(new StopIntake());
    // this.xboxDriver.getRightMiddleButton().onTrue(new PivotTurtle());
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoRunner.getAutonomousCommand();
  }

  // Reset encoders for auto
  public void resetForAuto(Pose2d pose) {
    swerveSubsystem.resetOdometry(pose);
  }

  public void prepareForAuto() {
    //Do nothing
  }



  // Override commands and switch to manual control
}
