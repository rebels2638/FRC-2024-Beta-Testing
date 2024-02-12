package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveSubsystemIO {
    @AutoLog
    public static class SwerveSubsystemIOInputs {
        public double[] pose = new double[3];
        public double yaw = 0;
        public double[] fieldVelocity = new double[3];
        public double[] robotVelocity = new double[3];
        public double pitch = 0;
        public double[] desiredModuleStates = new double[8];
        public double[] measuredModuleStates = new double[8];
        public double[] measuredChassisSpeeds = new double[3];
        public double[] desiredChassisSpeeds = new double[3];
    } 

    public default void updateInputs(SwerveSubsystemIOInputs inputs) {}

}
