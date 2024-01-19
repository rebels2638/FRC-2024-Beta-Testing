package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad;
        public double positionDeg;
        public double velocityDegSec;
        public double velocityRadSec;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
        
    }
    public default void setHightMeters(double goalPositionRad, double currentRadAngle) {
    }
    public default void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
    }
    public default void configureController(ElevatorFeedforward pff, PIDController pfb, 
                                                ElevatorFeedforward vff, PIDController vfb) {
    }
    public default void setVoltage(double voltage){
        
    };

    public abstract boolean reachedSetpoint(boolean isPositionalControll);

    public abstract void zeroHight();
    
}
