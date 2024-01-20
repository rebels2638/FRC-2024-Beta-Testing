package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private static final double kHightMetersPositionTolerance = .03;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    PIDController positionFeedBackController;
    ElevatorFeedforward positionFeedForwardController;

    PIDController velocityFeedBackController;
    ElevatorFeedforward velocityFeedForwardController;

    public Elevator(ElevatorIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(3, 0, 0);
        positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);
        positionFeedBackController.setTolerance(kHightMetersPositionTolerance);

        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedForwardController = new ElevatorFeedforward(0, 0, 0);


        io.configureController(positionFeedForwardController, positionFeedBackController);
    }

    @Override
    public void periodic() {
        io.configureController(positionFeedForwardController, positionFeedBackController);

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setHightMeters(double hightMeters) {
        io.setHightMeters(hightMeters, inputs.hightMeters);
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getHightMeters() {
        return inputs.hightMeters;
    }

    public void zeroHight() {
        io.zeroHight();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
