package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private static final double kRadPositionTolerance = Math.toRadians(8);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private boolean velocityControlmode;
    PIDController positionFeedBackController;
    ArmFeedforward positionFeedForwardController;

    PIDController velocityFeedBackController;
    ArmFeedforward velocityFeedForwardController;

    public Pivot(PivotIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(3, 0, 0);
        positionFeedForwardController = new ArmFeedforward(0, 0, 0);
        positionFeedBackController.setTolerance(kRadPositionTolerance);

        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedForwardController = new ArmFeedforward(0, 0, 0);


        io.configureController(positionFeedForwardController, positionFeedBackController,
            velocityFeedForwardController, velocityFeedBackController);
    }

    @Override
    public void periodic() {
        io.configureController(positionFeedForwardController, positionFeedBackController,
            velocityFeedForwardController, velocityFeedBackController);

        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
    }

    public void setDegAngle(double angle) {
        Logger.recordOutput("Pivot/desiredDegAngle", angle);
        io.setPosition(Math.toRadians(angle));
        return;
    }

    public void setVelocityControlMode(boolean b){  
        velocityControlmode = b;
    };

    public void setVelocitySetPoint(double setPoint){
        io.setVelocity(setPoint);
        return;
    }
    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getDegAngle() {
        return inputs.positionDeg;
    }

    public double getRadAngle() {
        return inputs.positionRad;
    }

    public void zeroAngle() {
        io.zeroAngle();
    }
    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }
}
