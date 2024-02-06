package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRadSec;
        public boolean inIntake; 
        public boolean reachedSetpoint;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {}

    public default void configureController(SimpleMotorFeedforward vff, PIDController vfb) {}

    public default void setVoltage(double voltage) {}
    
}
