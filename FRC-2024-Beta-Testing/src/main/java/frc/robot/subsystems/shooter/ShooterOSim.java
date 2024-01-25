package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterOSim extends SubsystemBase implements ShooterIO {

    double desiredVelocityRadSec = 0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = desiredVelocityRadSec;
    }

    @Override
    // sould be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        desiredVelocityRadSec = goalVelocityRadPerSec;
    } 

    @Override
    public void setVoltage(double voltage){
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
    }

    @Override
    public boolean reachedSetpoint() {
        return true;
    }

}