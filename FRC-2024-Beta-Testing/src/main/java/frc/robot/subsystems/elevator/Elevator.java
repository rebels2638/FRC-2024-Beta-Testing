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
        positionFeedBackController = new PIDController(0.0, 0.034, 0.073);
        //
        positionFeedForwardController = new ElevatorFeedforward(0.0, .31, 31);

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

    public void setHightMeters(double hightMeters) {
        Logger.recordOutput("Elevator/desiredHightMeters", hightMeters);
        io.setHightMeters(hightMeters, inputs.hightMeters);
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getHeightMeters() {
        return inputs.hightMeters;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
