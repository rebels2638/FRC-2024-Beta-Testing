package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    PIDController positionFeedBackController;
    ElevatorFeedforward positionFeedForwardController;
    ElevatorFeedforward climbFeedForwardController;

    // PIDController velocityFeedBackController;
    // ElevatorFeedforward velocityFeedForwardController; //Literally never gonna be used
    private static final double kPID_TOLERANCE_METERS = 0.01; //this is 1cm 
    private static final double kCLIMB_KG = 12;
    
    private double goalPositionMeters = 0;
    private boolean isShooterHeight = true;
    private boolean isClimbing = false;

    public Elevator(ElevatorIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(5, 0, 0); // 0 0 0 
        positionFeedForwardController = new ElevatorFeedforward(0.348, 0.24, 0); //0.33, 0.14, 0 
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

        if (isShooterHeight) {
            Logger.recordOutput("Elevator/desiredShooterHeight", goalPositionMeters);
            io.setHeightMeters(goalPositionMeters, isShooterHeight, isClimbing);
        }
        else {
            Logger.recordOutput("Elevator/desiredClimberHeight", goalPositionMeters);
            io.setHeightMeters(goalPositionMeters, isShooterHeight, isClimbing);
        }
        Logger.recordOutput("Elavator/isClimbing", isClimbing);
    
    }

    public void setHeightMeters(double goalPositionMeters, boolean isShooterHeight, boolean isClimbing) {
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
}
