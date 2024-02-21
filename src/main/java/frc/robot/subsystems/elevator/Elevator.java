package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private static ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    PIDController positionFeedBackController;
    ElevatorFeedforward positionFeedForwardController;
    ElevatorFeedforward climbFeedForwardController;

    // PIDController velocityFeedBackController;
    // ElevatorFeedforward velocityFeedForwardController; //Literally never gonna be used
    private static final double kPID_TOLERANCE_METERS = 0.01; //this is 1cm 
    private static final double kCLIMB_KG = 12;
    
    private static Elevator instance = null;
    private double goalPositionMeters = 0;
    private boolean isShooterHeight = true;
    private boolean isClimbing = false;
    private boolean isRaw = false;

    public Elevator(ElevatorIO io)  {
        Elevator.io = io;
        positionFeedBackController = new PIDController(8, 3, 0); // 12, 2, 0
        positionFeedForwardController = new ElevatorFeedforward(0.13, 0.05, 0); //0.33, 0.14, 0 
        climbFeedForwardController = new ElevatorFeedforward(0,0,0); //TODO: Tune later, use the raw elevator control or a setVoltage initially to give Build team measurements.
        
        // velocityFeedBackController = new PIDController(0, 0, 0);
        // velocityFeedForwardController = new ElevatorFeedforward(0,0, 0);
        
        positionFeedBackController.setTolerance(kPID_TOLERANCE_METERS);

        io.configureController(positionFeedForwardController, positionFeedBackController, climbFeedForwardController, kCLIMB_KG);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        if(isRaw){
            io.setHeightMeters(goalPositionMeters, false, false, isRaw);
        }
        else if (isShooterHeight) {
            Logger.recordOutput("Elevator/desiredShooterHeight", goalPositionMeters);
            io.setHeightMeters(goalPositionMeters, isShooterHeight, isClimbing, false);
        }
        else {
            Logger.recordOutput("Elevator/desiredClimberHeight", goalPositionMeters);
            io.setHeightMeters(goalPositionMeters, isShooterHeight, isClimbing, false);
        }
        Logger.recordOutput("Elavator/isClimbing", isClimbing);
    
    }

    public void setHeightMeters(double goalPositionMeters, boolean isShooterHeight, boolean isClimbing, boolean isRaw) {
        this.goalPositionMeters = goalPositionMeters;
        this.isShooterHeight = isShooterHeight;
        this.isClimbing = isClimbing;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getShooterHeightMeters() {
        return inputs.shooterHeightMeters;
    }
    public double getClimberHeightMeters(){
        return inputs.climberHeightMeters;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }
    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }
    public static Elevator getInstance(){
        if(instance == null){
            return new Elevator(Elevator.io);
        }
        return null;
    }
    //Sets and returns instance, the only reason why it returns is to just make our life easier.
    public static Elevator setInstance(Elevator inst){
        instance = inst;
        return inst;
    }
}
