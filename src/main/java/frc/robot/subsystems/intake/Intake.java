package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private static final double kVelocityRadSecTolerance = Math.toRadians(3);

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    PIDController velocityFeedBackController;
    SimpleMotorFeedforward velocityFeedForwardController;
    double desiredVelocityRadSec = 0;

    public Intake(IntakeIO io)  {
        this.io = io;
        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedBackController.setTolerance(kVelocityRadSecTolerance);
        velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);
        io.configureController(velocityFeedForwardController, velocityFeedBackController);
    }

    @Override
    public void periodic() {
        io.configureController(velocityFeedForwardController, velocityFeedBackController);

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Pivot/desiredVelocityRadSec", desiredVelocityRadSec);
        io.setVelocityRadSec(desiredVelocityRadSec, inputs.velocityRadSec);
    }

    public void setVelocityRadSec(double velo) {
        desiredVelocityRadSec = velo;
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getVelocityDegSec() {
        return Math.toDegrees(inputs.velocityRadSec);
    }

    public double getVelocityRadSec() {
        return inputs.velocityRadSec;
    }
    
    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }

    public boolean inIntake() {
        return inputs.inIntake;
    }
}
