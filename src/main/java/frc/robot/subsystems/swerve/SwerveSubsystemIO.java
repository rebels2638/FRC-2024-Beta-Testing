package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveSubsystemIO {
    @AutoLog
    public static class SwerveSubsystemIOInputs {
        public Pose2d pose = new Pose2d();
        public Rotation2d yaw = new Rotation2d();
        public ChassisSpeeds fieldVelocity = new ChassisSpeeds();
        public ChassisSpeeds robotVelocity = new ChassisSpeeds();
        public Rotation2d pitch = new Rotation2d();
        public double[] desiredModuleStates = new double[8];
        public double[] measuredModuleStates = new double[8];
        public double[] measuredChassisSpeeds = new double[3];
        public double[] desiredChassisSpeeds = new double[3];
    } 

    public default void updateInputs(SwerveSubsystemIOInputs inputs) {}

}
