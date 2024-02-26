// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorToggle;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.commands.shooter.ShooterWindReverse;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.climber.MoveClimberRaw;
import frc.robot.commands.compositions.CancelIntakeNote;
import frc.robot.commands.compositions.FeedAndHoldNote;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.ScoreAMP;
import frc.robot.commands.compositions.ShootNote;
import frc.robot.commands.compositions.ShootNoteTele;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberOSim;
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
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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

  public RobotContainer() {

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
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIOSim())); //Assigns the instance object(pointer) to the variable so no new changes are needed.
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIOSim()));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIOSim()));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIOSim())); 

        climberSubsystem = Climber.setInstance(new Climber(new ClimberOSim()));
        visionSubsystem = new PoseLimelight(new PoseLimelightIOSim());
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon"), visionSubsystem);

        break;
      
      case REPLAY:
        swerveSubsystem.setIO(new SwerveSubsystemIO() {});
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIO(){}));
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIO(){}));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIO(){}));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIO(){}));  
        
        climberSubsystem = Climber.setInstance(new Climber(new ClimberIO(){}));

        visionSubsystem = new PoseLimelight(new PoseLimelightIO() {});
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon"), visionSubsystem);

        break;
        
      default:
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));
        shooterSubsystem = Shooter.setInstance(new Shooter(new ShooterIOFalcon()));
        intakeSubsystem = Intake.setInstance(new Intake(new IntakeIONeo()));
        elevatorSubsystem = Elevator.setInstance(new Elevator(new ElevatorIOFalcon()));
        pivotSubsystem = Pivot.setInstance(new Pivot(new PivotIONeo()));
        climberSubsystem = Climber.setInstance(new Climber(new ClimberIOFalcon()));

        visionSubsystem = new PoseLimelight(new PoseLimelightIOReal());
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon"), visionSubsystem);

        break;
    }

    autoRunner = new AutoRunner(swerveSubsystem);

    closedFieldAbsoluteDrive = new AbsoluteFieldDrive(swerveSubsystem,
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), false);

    NamedCommands.registerCommand("MoveElevatorAMP", new MoveElevatorAMP());
    NamedCommands.registerCommand("MoveElevatorTurtle", new MoveElevatorTurtle());
    NamedCommands.registerCommand("ShooterWindUp", new ShooterWindup());
    NamedCommands.registerCommand("RollIntakeIn", new RollIntakeIn());
    NamedCommands.registerCommand("StopIntake", new StopIntake());
    NamedCommands.registerCommand("IntakeNote", new IntakeNote());
    NamedCommands.registerCommand("ShooterStop", new ShooterStop());
    NamedCommands.registerCommand("ShooterWindReverse", new ShooterWindReverse());
    NamedCommands.registerCommand("ShootNote", new ShootNote());

    swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);
    climberSubsystem.setDefaultCommand(new MoveClimberRaw(climberSubsystem ,xboxTester));
    xboxTester.getAButton().onTrue(new PivotToTorus());
    xboxTester.getBButton().onTrue(new MoveElevatorAMP());
    xboxTester.getYButton().onTrue(new MoveElevatorTurtle());
    xboxTester.getXButton().onTrue(new PivotTurtle());
    xboxTester.getLeftBumper().onTrue(new InstantCommand(()-> climberSubsystem.zeroHeight()));

    //TrevorBallshack Controls
    swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);
    this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    this.xboxDriver.getLeftBumper().onTrue(new IntakeNote());
    this.xboxDriver.getRightBumper().onTrue(new CancelIntakeNote());

    // //Michaelangelo controls
    this.xboxOperator.getLeftBumper().onTrue(new ShooterStop());
    this.xboxOperator.getRightBumper().onTrue(new ShooterWindup());
    this.xboxOperator.getXButton().onTrue(new MoveElevatorToggle());
    this.xboxOperator.getYButton().onTrue(new ScoreAMP());
    this.xboxOperator.getAButton().onTrue(new ShootNoteTele());
    this.xboxOperator.getBButton().onTrue(new FeedAndHoldNote());
    
    Shuffleboard.getTab("Auto").add("Zero Swerve", new InstantCommand(() -> swerveSubsystem.zeroGyro()));

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
    
  }

  //TODO: Do later once we are done with all of our priorities.
  // public void setInstances(){
  //   pivotSubsystem = Pivot.getInstance();
  //   elevatorSubsystem = Elevator.getInstance();
  //   shooterSubsystem = Shooter.getInstance();
  //   intakeSubsystem = Intake.getInstance();
  //   shooterSubsystem =  Shooter.getInstance();
  // }

  // Override commands and switch to manual control
}
