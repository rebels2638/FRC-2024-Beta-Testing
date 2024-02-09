
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRadSec;
    }

    public abstract void updateInputs(ShooterIOInputs inputs);

    public abstract void setVelocityRadSec(double goalVelocityRadPerSec);

    public abstract void configureController(SimpleMotorFeedforward vff, PIDController vfb);

    public abstract void setVoltage(double voltage);

    public abstract boolean reachedSetpoint();
    
}
