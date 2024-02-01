package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double heightMeters;
        public double voltageOut;
    }

    public abstract void updateInputs(ElevatorIOInputs inputs);

    public abstract void setHeightMeters(double goalPositionRad, double currentHightMeters);

    public abstract void configureController(ElevatorFeedforward pff, PIDController pfb);

    public abstract void setVoltage(double voltage);

    public abstract boolean reachedSetpoint();

    public abstract void zeroHeight();
    
}
