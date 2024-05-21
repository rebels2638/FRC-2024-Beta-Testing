package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private static final double kVelocityRadSecTolerance = Math.toRadians(.1);

    private static IntakeIO io;
    private static Intake instance;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    PIDController velocityFeedBackController;
    SimpleMotorFeedforward velocityFeedForwardController;
    double desiredVelocityRadSec = 0;



    public Intake(IntakeIO io)  {
        Intake.io = io;   
        velocityFeedBackController = new PIDController(0.1,0,0.0); // 1 0 0
        velocityFeedBackController.setTolerance(kVelocityRadSecTolerance);
        velocityFeedForwardController = new SimpleMotorFeedforward(0.3, .5, 0);
        io.configureController(velocityFeedForwardController, velocityFeedBackController);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/desiredVelocityRadSec", desiredVelocityRadSec);
        
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

    public boolean inIntake() {
        return inputs.inIntake;
    }
    public void setIntakeStatus(boolean s){

    }
    public static Intake getInstance(){
        if(Intake.instance == null){
            return new Intake(Intake.io);
        }
        return instance;
    }
    public static Intake setInstance(Intake inst){
        Intake.instance = inst;
        return inst;
    }
}
