package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private static final double kVelocityRadSecTolerance = Math.toRadians(3);

    private final ShooterIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    PIDController velocityFeedBackController;
    SimpleMotorFeedforward velocityFeedForwardController;

    public Shooter(ShooterIO io)  {
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
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocityRadSec(double velo) {
        Logger.recordOutput("Shooter/desiredVelocityRadSec", velo);
        io.setVelocityRadSec(velo, inputs.velocityRadSec);
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
        return io.reachedSetpoint();
    }

}
