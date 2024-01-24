package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double hightMeters;
    }

    public abstract void updateInputs(ElevatorIOInputs inputs);

    public abstract void setHightMeters(double goalPositionRad, double currentHightMeters);

    public abstract void configureController(ElevatorFeedforward pff, PIDController pfb);

    public abstract void setVoltage(double voltage);

    public abstract boolean reachedSetpoint();

    public abstract void zeroHeight();
    
}
