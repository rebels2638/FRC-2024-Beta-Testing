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
    inputs.pose[0] = swerveDrive.getPose().getTranslation().getX();
    inputs.pose[1] = swerveDrive.getPose().getTranslation().getY();
    inputs.pose[2] = swerveDrive.getPose().getRotation().getRadians();

    inputs.yaw = swerveDrive.getYaw().getRadians();
    inputs.pitch = swerveDrive.getPitch().getRadians();
    
    inputs.fieldVelocity[0] = swerveDrive.getFieldVelocity().vxMetersPerSecond;
    inputs.fieldVelocity[1] = swerveDrive.getFieldVelocity().vyMetersPerSecond;
    inputs.fieldVelocity[2] = swerveDrive.getFieldVelocity().omegaRadiansPerSecond;
    
    inputs.robotVelocity[0] = swerveDrive.getRobotVelocity().vxMetersPerSecond;
    inputs.robotVelocity[1] = swerveDrive.getRobotVelocity().vyMetersPerSecond;
    inputs.robotVelocity[2] = swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    
    inputs.desiredModuleStates = SwerveDriveTelemetry.desiredStates;
    inputs.measuredModuleStates = SwerveDriveTelemetry.measuredStates;

    inputs.desiredChassisSpeeds = SwerveDriveTelemetry.desiredChassisSpeeds;
    inputs.measuredChassisSpeeds = SwerveDriveTelemetry.measuredChassisSpeeds;
  }
  
}
