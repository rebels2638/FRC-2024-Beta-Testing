package frc.robot.subsystems.poseLimelight;

import org.littletonrobotics.junction.AutoLog;

public interface PoseLimelightIO {
    @AutoLog
    public static class PoseLimelightIOInputs {
        public double[] botpose_wpiblue;
        public double[] botpose_wpired;
        public double tv;
        public double cl;
        public double ta;
        public double ambiguity;
    }

    public default void updateInputs(PoseLimelightIOInputs inputs) {}
}