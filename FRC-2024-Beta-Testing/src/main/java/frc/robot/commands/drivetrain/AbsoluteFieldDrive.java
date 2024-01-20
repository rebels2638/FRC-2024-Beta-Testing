package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;

import java.util.List;
import java.util.function.DoubleSupplier;

import frc.robot.lib.swervelib.SwerveController;
import frc.robot.lib.swervelib.math.SwerveMath;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AbsoluteFieldDrive extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY, heading;
  private final boolean isOpenLoop;
  private boolean resetRotation = false;
  private double lastHeading = 0;
  private double lastTime = 0;
  private Rotation2d desiredHeading = new Rotation2d(0);
  PIDController translationPID = new PIDController(3,0, 0);
  // PIDController translationPID = new PIDController(0,0, 0);

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier heading, boolean isOpenLoop)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
    this.isOpenLoop = isOpenLoop;


    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    //TranslationalVelocity Correction Via PID controls.
    translationPID.setTolerance(0.1); //Note : Moved the PID 
    double xCorrection = 0;
    double yCorrection = 0;
    if (Math.abs(swerve.getFieldVelocity().vxMetersPerSecond) + Math.abs(swerve.getFieldVelocity().vyMetersPerSecond) > 1 && Math.abs(Math.toDegrees(swerve.getFieldVelocity().omegaRadiansPerSecond)) > 60) {
      translationPID.setSetpoint(vX.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC);
      xCorrection = translationPID.calculate(swerve.getFieldVelocity().vxMetersPerSecond);
  
      translationPID.setSetpoint(vY.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC);
      yCorrection = translationPID.calculate(swerve.getFieldVelocity().vyMetersPerSecond);
    }
    

    SmartDashboard.putNumber("swerve/yCorrection", yCorrection);
    SmartDashboard.putNumber("swerve/xCorrection", xCorrection);

    desiredHeading = new Rotation2d(heading.getAsDouble() * Math.PI);
    ChassisSpeeds desiredSpeeds; 
    if (!resetRotation) {
      desiredSpeeds = new ChassisSpeeds(vX.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC + xCorrection,
        vY.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC + yCorrection, 
        heading.getAsDouble() * Math.toRadians(Constants.Drivebase.MAX_DEG_SEC_ROTATIONAL_VELOCITY));
    } 
    else {
      desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), desiredHeading);
      desiredSpeeds = new ChassisSpeeds(vX.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC + xCorrection,
      vY.getAsDouble() * Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC + yCorrection, desiredSpeeds.omegaRadiansPerSecond);
    }

    // Limit velocity to prevent tipsy turby  
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                           Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                           swerve.getSwerveDriveConfiguration());

    
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, isOpenLoop);
    // lastHeading = swerve.getHeading().getRadians();
    lastHeading = desiredHeading.getRadians();
    lastTime = Timer.getFPGATimestamp();
    return;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

  public void toggleRotationMode() {
      resetRotation = !resetRotation;
      // resetRotation = false;
  }


}