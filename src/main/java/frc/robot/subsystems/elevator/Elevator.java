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

    PIDController velocityFeedBackController;
    ElevatorFeedforward velocityFeedForwardController;
    private static final double kPID_TOLERANCE_METERS = 0.01;
    private static final double kCLIMB_KG = 12;

    public Elevator(ElevatorIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(6, 0, 0); // 0 0 0 
        //
        positionFeedForwardController = new ElevatorFeedforward(0.33, 0.14, 0); //0.008 0.31 31

        // dont use
        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedForwardController = new ElevatorFeedforward(0,0, 0);
        
        positionFeedBackController.setTolerance(kPID_TOLERANCE_METERS);

        io.configureController(positionFeedForwardController, positionFeedBackController, kCLIMB_KG);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setheightMeters(double goalPositionMeters, boolean isShooterheight, boolean isClimbing) {
        if (isShooterheight) {
            Logger.recordOutput("Elevator/desiredShooterheight");
            io.setheightMeters(goalPositionMeters, inputs.shooterheightMeters, isShooterheight, isClimbing);
        }
        // else {
        //     Logger.recordOutput("Elevator/desiredClimberheight");
        //     io.setheightMeters(goalPositionMeters, inputs.climberheightMeters, isShooterheight, isClimbing);
        // }
        // Logger.recordOutput("Elavator/isClimbing", isClimbing);
    
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getShooterHeightMeters() {
        return inputs.shooterheightMeters;
    }
    public double getClimberHeightMeters(){
        return inputs.climberheightMeters;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
