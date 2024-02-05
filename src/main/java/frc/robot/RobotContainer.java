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
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorTurtle;
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
import frc.robot.Utils.Constants.OperatorConstants;
// import frc.robot.commands.automation.AutoAlign;
import frc.robot.commands.drivetrain.AbsoluteDrive;
// import frc.robot.commands.pivot.RollIntake;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.audio.*;
import frc.robot.subsystems.audio.AudioPlayer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIONeo;
// import frc.robot.subsystems.elevator.ElevatorIOSim;;



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
  
  // // Robot Subsystems
  // private final AprilTagVisionIO aprilTagVisionIO;
  // private final AprilTagVision aprilTagVision;
  private SwerveSubsystem swerveSubsystem;
  // private final TeleopDrive closedFieldRel;
  // private final AbsoluteDrive closedAbsoluteDrive;
  private final AbsoluteFieldDrive closedFieldAbsoluteDrive;
  
  //private final Intake intakeSubsystem;
  // private final Pivot pivotSubsystem;

  // Auto
 private final AutoRunner autoRunner;
 // private final int[] autoAlignTargetNum = {0};
  //private final SmartDashboardLogger smartDashboardLogger = new SmartDashboardLogger();
  // private AprilTagVision aprilTagVision;

  // private final Elevator elevator;
  // private final Pivot pivot;
  //private final AudioPlayer aPlayer;

  public RobotContainer() {


    // if (RobotBase.isReal()) {
    //   elevator = new Elevator(new ElevatorIONeo());
    // }
    // else {
    //   // elevator = new Elevator(new ElevatorIOSim());
    //   elevator = new Elevator(new ElevatorIONeo());
    // }


    // if (RobotBase.isReal()) {
    //   pivot = new Pivot(new PivotIONeo());
    // }
    // else {
    //   pivot = new Pivot(new PivotIOSim());
    // }

    

    // AprilTagVisionIO aprilTagVisionIO = new AprilTagVisionIOSim();
  // System.out.println("Is directory? : " +new File("C:/Users/RebelRobotics/Documents/2024/FRC-2024-Beta-Testing/FRC-2024-Beta-Testing/src/main/deploy/swerve/falcon").isDirectory());
  // System.out.println("Deploy Directory : " + Filesystem.getDeployDirectory());
  // System.out.println("Is File? : " + new File("C:/Users/RebelRobotics/Documents/2024/FRC-2024-Beta-Testing/FRC-2024-Beta-Testing/src/main/deploy/swerve/falcon").isFile());

    // swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"/swerve/falcon") /* , new AprilTagVision(aprilTagVisionIO) */);
    // System.out.println(new File(Filesystem.getDeployDirectory(),"/swerve/falcon").isFile());
    
   //swerveSubsystem = new SwerveSubsystem(new File("C:/Users/RebelRobotics/Documents/2024/FRC-2024-Beta-Testing/FRC-2024-Beta-Testing/src/main/java/deploy/swerve/falcon") /* , new AprilTagVision(aprilTagVisionIO)*/);
  //  aPlayer = new AudioPlayer();
    if (RobotBase.isReal()) {
      // aprilTagVisionIO = new AprilTagVisionIOReal();
      System.out.println("Is directory? : " + new File(Filesystem.getDeployDirectory(),"/swerve/falcon").isDirectory());
      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"/swerve/falcon") /* , new AprilTagVision(aprilTagVisionIO) */);
    }else{
      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "/swerve/falcon"));
    }
    
    autoRunner = new AutoRunner(swerveSubsystem);
    
    //PivotIONeo pivotIO = new PivotIONeo();
    // if (RobotBase.isReal()) {
    //   pivotIO = new PivotIONeo();
    // }
    // pivotSubsystem = new Pivot(pivotIO);
    // PivotController pivotController;
    //intakeSubsystem = new Intake();
    
    // Instantiate our controllers with proper ports.
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    //pivotController = new PivotController(pivotSubsystem, xboxOperator);
    // try {
    //   JsonChanger jsonChanger = new JsonChanger();
    //  }
    // catch (IOException e) {
    //   e.printStackTrace();;
    // }

    // Controller Throttle Mappings
    // this.drive.setDefaultCommand(new FalconDrive(drive, limelight, xboxDriver));
    
    // closedAbsoluteDrive = new AbsoluteDrive(swerveSubsystem, 
    // () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(), OperatorConstants.LEFT_X_DEADBAND),
    // () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
    // () -> xboxDriver.getRightX(),
    // () -> xboxDriver.getRightY(), false);

    closedFieldAbsoluteDrive = new AbsoluteFieldDrive(swerveSubsystem,
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftY(),OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getLeftX(),OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(-xboxDriver.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), false); //TODO: tune the rightX value constant



    /* 
    closedFieldRel = new TeleopDrive(
    swerveSubsystem,
    () -> MathUtil.applyDeadband(xboxDriver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(xboxDriver.getRightX(),OperatorConstants.RIGHT_X_DEADBAND), () -> true, false, true, xboxDriver);
    */

    //System.out.println(xboxDriver.getRightX()+","+xboxDriver.getRightY());

    // swerveSubsystem.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);

    swerveSubsystem.setDefaultCommand(closedFieldAbsoluteDrive);
    
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
    //.xboxDriver.getAButton().onTrue(new AutoAlign(swerveSubsystem, () -> autoAlignTargetNum[0], xboxDriver));
   // this.xboxDriver.getBButton().onTrue(new InstantCommand(() -> closedFieldAbsoluteDrive.toggleRotationMode()) );
    // this.xboxDriver.getXButton().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    //this.xboxDriver.getAButton().onTrue(new InstantCommand(() -> swerveSubsystem.lock()));
    //this.xboxDriver.getYButton().onTrue(new PickUpCube(intakeSubsystem, pivotSubsystem));
    //this.xboxDriver.getYButton().onTrue(new RollIntake(intakeSubsystem));
    
    // this.xboxDriver.getRightStick.onTrue(new InstantCommand(() -> ))
    //this.xboxDriver.getYButton().onTrue(new InstantCommand(() -> pivotSubsystem.zeroAngle()));
    // this.xboxDriver.getYButton().onTrue(new InstantCommand(() -> pivotSubsystem.zeroAngle()));
    // this.pivotSubsystem.setDefaultCommand(pivotController);
    // this.xboxDriver.getAButton().onTrue(new Turtle(pivotSubsystem));
    // this.xboxDriver.getBButton().onTrue(new PivotToCube(pivotSubsystem));

    // //TODO: ELEVATOR
    // this.xboxDriver.getAButton().onTrue(new MoveElevatorAMP(elevator));
    // this.xboxDriver.getBButton().onTrue(new MoveElevatorTurtle(elevator));
    // this.xboxDriver.getYButton().onTrue(new InstantCommand(() -> elevator.zeroHeight()));

    // this.xboxDriver.getXButton().onTrue(new AutoAlignAMP(swerveSubsystem));
    // this.xboxDriver.getYButton().onTrue(new AutoAlignTrap(swerveSubsystem));
    // this.xboxOperator.getAButton().onTrue(new playMusic(aPlayer));

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
  public Command getAutonomousCommand() { // Command

    return autoRunner.getAutonomousCommand();
    // return null;
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

  // Override commands and switch to manual control
}