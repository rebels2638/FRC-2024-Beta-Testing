package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double positionRad;
        public double positionDeg;
        public double velocityDegSec;
        public double velocityRadSec;
        public boolean reachedSetpoint;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setPosition(double goalPositionRad) {}

    public default void setVelocity(double goalVelocityRadPerSec) {}

    public default void configureController(ArmFeedforward pff, PIDController pfb) {}
    
    public default void setVoltage(double voltage) {}

    public default void zeroAngle() {}

    public default void toggleMode() {}

    public static Pivot getInstance(){
        return null;
    }
    public default void TorusAngleReset(){}

    
}
