
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface ShooterIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRadSec;
    }

    public abstract void updateInputs(IntakeIOInputs inputs);

    public abstract void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec);

    public abstract void configureController(SimpleMotorFeedforward vff, PIDController vfb);

    public abstract void setVoltage(double voltage);

    public abstract boolean reachedSetpoint();
    
}
