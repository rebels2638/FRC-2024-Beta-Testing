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

    // public void setID();

    public default double getID(){return 0.0;}

    public default double getV(){return 0.0;}

    public default double getX(){return 0.0;}
    
    public default double getY(){return 0.0;}

    public default double getA(){return 0.0;}

    public default void setInstance(PoseLimelight p) {}

    public default PoseLimelight getInstance() {return null;}
}
