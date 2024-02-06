package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveSubsystemIO {
    @AutoLog
    public static class SwerveSubsystemIOInputs {
        public Pose2d pose;
        public Rotation2d yaw;
        public ChassisSpeeds fieldVelocity;
        public ChassisSpeeds robotVelocity;
        public Rotation2d pitch;
        public double[] desiredModuleStates;
        public double[] measuredModuleStates;
        public double[] measuredChassisSpeeds;
        public double[] desiredChassisSpeeds;
    } 

    public default void updateInputs(SwerveSubsystemIOInputs inputs) {}

}
