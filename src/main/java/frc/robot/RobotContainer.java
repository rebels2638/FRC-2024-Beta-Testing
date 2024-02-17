// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import java.io.File;
import java.util.Optional;
// import frc.robot.commands.pivot.PickUpCube;
// import frc.robot.commands.pivot.PivotController;
// import frc.robot.commands.pivot.PivotToCube;
// import frc.robot.commands.pivot.RollIntake;
// import frc.robot.commands.pivot.Turtle;
import frc.robot.commands.drivetrain.AbsoluteFieldDrive;
import frc.robot.commands.elevator.ElevatorControlRaw;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotController;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.commands.shooter.ShooterToggle;
import frc.robot.commands.shooter.ShooterWindReverse;
import frc.robot.commands.Intake.IntakeToggle;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.commands.Intake.RollIntakeOut;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.audio.*;
import frc.robot.commands.compositions.FeedAndHoldNote;
import frc.robot.commands.compositions.IntakeNote;
import frc.robot.commands.compositions.ScoreAMP;
import frc.robot.subsystems.audio.AudioPlayer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIONeo;
// import frc.robot.subsystems.elevator.ElevatorIOSim;;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;
// import frc.robot.subsystems.limelight.PoseLimelight;
// import frc.robot.subsystems.limelight.PoseLimelightIO;
// import frc.robot.subsystems.limelight.PoseLimelightIOReal;
// import frc.robot.subsystems.limelight.PoseLimelightIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIONeo;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOFalcon;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.commands.shooter.ShooterStop;

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
  // private final AprilTagVisionIO aprilTagVisionIO;
  // private final AprilTagVision aprilTagVision;
  private SwerveSubsystem swerveSubsystem;
  private final AbsoluteFieldDrive closedFieldAbsoluteDrive;
  private final PivotController pivotController;
  
  private  Intake intakeSubsystem;
  private  Shooter shooterSubsystem; 
  private final AutoRunner autoRunner;
  // private final int[] autoAlignTargetNum = {0};
  // private final SmartDashboardLogger smartDashboardLogger = new SmartDashboardLogger();
  // private AprilTagVision aprilTagVision;

  private  Elevator elevatorSubsystem;
  // private final PoseLimelight poseLimelightSubsystem; 
  private  Pivot pivotSubsystem;
  // private final AudioPlayer aPlayer;

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
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon")/* , poseLimelightSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));

        intakeSubsystem = new Intake(new IntakeIOSim() {});

        shooterSubsystem = new Shooter(new ShooterIOSim());

        elevatorSubsystem = new Elevator(new ElevatorIOSim());

        pivotSubsystem = new Pivot(new PivotIOSim());
        break;
      
      case REPLAY:
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon")/* , poseLimelightSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIO() {});
        
        shooterSubsystem = new Shooter(new ShooterIO(){});

        intakeSubsystem = new Intake(new IntakeIO() {});
        pivotSubsystem = new Pivot(new PivotIO() {});

        elevatorSubsystem = new Elevator(new ElevatorIO() {});
        break;
        
      default:
        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"/swerve/falcon")/* , poseLimelightSubsystem*/);
        swerveSubsystem.setIO(new SwerveSubsystemIORunning(swerveSubsystem.getSwerveDrive()));

        shooterSubsystem = new Shooter(new ShooterIOFalcon(){});

        intakeSubsystem = new Intake(new IntakeIONeo() {});

        elevatorSubsystem = new Elevator(new ElevatorIONeo());

        pivotSubsystem = new Pivot(new PivotIONeo());
        break;
    }

    autoRunner = new AutoRunner(swerveSubsystem);

    pivotController = new PivotController(pivotSubsystem, xboxOperator);

    closedFieldAbsoluteDrive = new AbsoluteFieldDrive(swerveSubsystem,
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), false);

    swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);

    // // shooterSubsystem.setDefaultCommand(new ShooterToggle(shooterSubsystem, xboxDriver));
    // // intakeSubsystem.setDefaultCommand(new IntakeToggle(intakeSubsystem, pivotSubsytem, xboxDriver));
    // // elevatorSubsystem.setDefaultCommand(new ElevatorControlRaw(elevatorSubsystem, xboxDriver));
    
    this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    // this.xboxDriver.getAButton().onTrue(new InstantCommand(() -> swerveSubsystem.lock()));
    // this.xboxDriver.getYButton().onTrue(new PickUpCube(intakeSubsystem, pivotSubsystem));
    this.xboxOperator.getYButton().onTrue(new RollIntakeIn(intakeSubsystem, pivotSubsystem));
    this.xboxOperator.getAButton().onTrue(new StopIntake(intakeSubsystem));
    this.xboxOperator.getXButton().onTrue(new RollIntakeOut(intakeSubsystem));
    this.xboxOperator.getBButton().onTrue(new ScoreAMP(shooterSubsystem, intakeSubsystem, pivotSubsystem, elevatorSubsystem));

    
    // this.xboxDriver.getLeftBumper().onTrue(new PivotToTorus(pivotSubsytem));
    // this.xboxDriver.getRightBumper().onTrue(new PivotTurtle(pivotSubsytem));
    // this.xboxDriver.getAButton().onTrue(new InstantCommand(()-> pivot.toggleMode()));
    // this.xboxOperator.getXButton().onTrue(new InstantCommand(() -> pivotSubsytem.zeroAngle()));

    // this.xboxOperator.getLeftBumper().onTrue(new ShooterStop(shooterSubsystem));
    // this.xboxOperator.getRightBumper().onTrue(new ShooterWindup(shooterSubsystem));

    this.xboxDriver.getAButton().onTrue(new IntakeNote(intakeSubsystem, pivotSubsystem));
    // // this.xboxDriver.getRightStick.onTrue(new InstantCommand(() -> ))
    // // this.xboxDriver.getYButton().onTrue(new InstantCommand(() -> pivotSubsystem.zeroAngle()));
    // // this.pivot.setDefaultCommand(pivotController);
    // this.xboxDriver.getLeftBumper().onTrue(new PivotTurtle(pivotSubsystem));
    // this.xboxDriver.getRightBumper().onTrue(new PivotToTorus(pivotSubsystem)); 



    // //TODO: ELEVATOR
    // this.xboxDriver.getAButton().onTrue(new MoveElevatorAMP(elevatorSubsystem));
    this.xboxDriver.getBButton().onTrue(new MoveElevatorTurtle(elevatorSubsystem));
    this.xboxDriver.getYButton().onTrue(new InstantCommand(() -> elevatorSubsystem.zeroHeight()));
    
    

    // this.xboxDriver.getXButton().onTrue(new AutoAlignAMP(swerveSubsystem));
    // this.xboxDriver.getYButton().onTrue(new AutoAlignTrap(swerveSubsystem));
    // this.xboxOperator.getAButton().onTrue(new playMusic(aPlayer));

    // this.xboxDriver.getRightBumper().onTrue(new InstantCommand(() ->  {
    //     if (autoAlignTargetNum[0] > 0) {
    //       autoAlignTargetNum[0]--;
    //     }
    //   } 
    // ));
    // this.xboxDriver.getLeftBumper().onTrue(new InstantCommand(() -> {
    //   if (autoAlignTargetNum[0] < 8) {
    //     autoAlignTargetNum[0]++;
    //   }
    // }));
    //xboxDriver.getAButton().onTrue(new AutoAlign(swerveSubsystem, () -> autoAlignTargetNum[0], xboxDriver));
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

  public static Boolean isRed(){
    try{
    Optional<Alliance> id = DriverStation.getAlliance();
    if(id.equals(AllianceStationID.Blue1) || id.equals(AllianceStationID.Blue2) || id.equals(AllianceStationID.Blue3)){
      return false;
    }
  }catch(Exception e){
    System.out.println(e);
  }
    return true;
  }
  // public void setInstances(){
  //   pivotSubsystem = Pivot.getInstance();
  //   elevatorSubsystem = Elevator.getInstance();
  //   shooterSubsystem = Shooter.getInstance();
  //   intakeSubsystem = Intake.getInstance();
  //   shooterSubsystem =  Shooter.getInstance();
  // }

  // Override commands and switch to manual control
}
