package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;

public interface PoseLimelightIO {
    @AutoLog
    public static class PoseLimelightIOInputs {
        public double[] botpose_wpiblue;
        public double[] botpose_wpired;
        public double tv, tx, ty, ta;
        public double cl;
    }

    public default void updateInputs(PoseLimelightIOInputs inputs) {}

    public void setID();

    public double getID();

    public double getV();

    public double getX();
    
    public double getY();

    public double getA();
}
