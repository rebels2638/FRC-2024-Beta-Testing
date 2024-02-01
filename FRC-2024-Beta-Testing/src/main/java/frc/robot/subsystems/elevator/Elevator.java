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

    public Elevator(ElevatorIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(8, 0, 0); // 0 0 0 
        //
        positionFeedForwardController = new ElevatorFeedforward(0.33, 0.14, 0); //0.008 0.31 31

        // dont use
        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedForwardController = new ElevatorFeedforward(0,0, 0);
        
        positionFeedBackController.setTolerance(kPID_TOLERANCE_METERS);

        io.configureController(positionFeedForwardController, positionFeedBackController);
    }

    @Override
    public void periodic() {
        io.configureController(positionFeedForwardController, positionFeedBackController);

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setHeightMeters(double heightMeters) {
        Logger.recordOutput("Elevator/desiredHeightMeters", heightMeters);
        io.setHeightMeters(heightMeters, inputs.heightMeters);
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getHeightMeters() {
        return inputs.heightMeters;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
