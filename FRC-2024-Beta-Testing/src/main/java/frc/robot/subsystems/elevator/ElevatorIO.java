package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double hightMeters;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
        
    }
    public default void setHightMeters(double goalPositionRad, double currentHightMeters) {
    }

    public default void configureController(ElevatorFeedforward pff, PIDController pfb) {
    }
    public default void setVoltage(double voltage){
        
    };

    public abstract boolean reachedSetpoint();

    public abstract void zeroHight();
    
}
