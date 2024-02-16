package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private static final double kVelocityRadSecTolerance = Math.toRadians(0);

    private static ShooterIO io;
    private static Shooter instance = null;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    PIDController velocityFeedBackController;
    SimpleMotorFeedforward velocityFeedForwardController;

    double desiredVelocityRadSec = 0;
    public Shooter(ShooterIO io)  {
        this.io = io;
        velocityFeedBackController = new PIDController(0.68, 0, 0.2);
        velocityFeedBackController.setTolerance(kVelocityRadSecTolerance);
        velocityFeedForwardController = new SimpleMotorFeedforward(0.14, 0, 0);
        io.configureController(velocityFeedForwardController, velocityFeedBackController);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/desiredVelocityRadSec", desiredVelocityRadSec);
        io.setVelocityRadSec(desiredVelocityRadSec);
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

    public boolean inShooter() {
        return inputs.inShooter;
    }
    public static Shooter getInstance(){
        if(instance == null){
            return new Shooter(Shooter.io);
        }
        return null;
    }


}
