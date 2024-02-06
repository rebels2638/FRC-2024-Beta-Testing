package frc.robot.subsystems.swerve;

import frc.robot.lib.swervelib.SwerveDrive;
import frc.robot.lib.swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystemIORunning implements SwerveSubsystemIO{
  private final SwerveDrive swerveDrive;
  public SwerveSubsystemIORunning(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void updateInputs(SwerveSubsystemIOInputs inputs) {
    inputs.pose = swerveDrive.getPose();
    inputs.yaw = swerveDrive.getYaw();
    inputs.pitch = swerveDrive.getPitch();
    
    inputs.fieldVelocity = swerveDrive.getFieldVelocity();
    inputs.robotVelocity = swerveDrive.getRobotVelocity();
    
    inputs.desiredModuleStates = SwerveDriveTelemetry.desiredStates;
    inputs.measuredModuleStates = SwerveDriveTelemetry.measuredStates;

    inputs.desiredChassisSpeeds = SwerveDriveTelemetry.desiredChassisSpeeds;
    inputs.measuredChassisSpeeds = SwerveDriveTelemetry.measuredChassisSpeeds;
  }
  
}
