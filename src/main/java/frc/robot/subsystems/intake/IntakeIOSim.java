package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOSim extends SubsystemBase implements IntakeIO {

    double desiredVelocityRadSec = 0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadSec = desiredVelocityRadSec;
        inputs.inIntake = true;
        inputs.reachedSetpoint = true;
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

}