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
    }

    public abstract void updateInputs(PivotIOInputs inputs);

    public abstract void setPosition(double goalPositionRad, double currentRadAngle);

    public abstract void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec);

    public abstract void configureController(ArmFeedforward pff, PIDController pfb, 
                                                    ArmFeedforward vff, PIDController vfb);
    public abstract void setVoltage(double voltage);

    public abstract boolean reachedSetpoint(boolean isPositionalControll);

    public abstract void zeroAngle();
    
}
