
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double velocityRadSec;
        public boolean reachedSetpoint;
        public boolean inShooter;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVelocityRadSec(double goalVelocityRadPerSec) {}

    public default void configureController(SimpleMotorFeedforward vff, PIDController vfb) {}

    public default void setVoltage(double voltage) {}
}
