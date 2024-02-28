package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;

public class Climber extends SubsystemBase{

    private static ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    PIDController positionFeedBackController;
    ElevatorFeedforward positionFeedForwardController;
    private double goalPositionMeters = 0; 

    // PIDController velocityFeedBackController;
    // ElevatorFeedforward velocityFeedForwardController; //Literally never gonna be used
    private static final double kPID_TOLERANCE_METERS = 0.01; //this is 1cm 
    private static final double kCLIMB_KG = 12;

    private static Climber instance = null;
            

    public Climber(ClimberIO io)  {
        Climber.io = io;
        positionFeedBackController = new PIDController(6, 0, 0); // 0 0 0 
        positionFeedForwardController = new ElevatorFeedforward(0, 0, 0); //0.008 0.31 31
        
        
        positionFeedBackController.setTolerance(kPID_TOLERANCE_METERS);

        io.configureController(positionFeedForwardController, positionFeedBackController);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        io.setHeightMeters(goalPositionMeters);

    }

    public void setHeightMeters(double goalPositionMeters) {
        Logger.recordOutput("Climber/desiredClimberHeight");
        // io.setHeightMeters(goalPositionMeters); //Duplicate method call
        this.goalPositionMeters = goalPositionMeters;

        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getClimberHeightMeters() {
        return inputs.climberHeightMeters;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }
    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }
    public static Climber getInstance(){
     if(instance == null){
     return new Climber(Climber.io);
    }
     return instance;
    }
    public static Climber setInstance(Climber inst){
        instance = inst;
        return inst;
    }
}
