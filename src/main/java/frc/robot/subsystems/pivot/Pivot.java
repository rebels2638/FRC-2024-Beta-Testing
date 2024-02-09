package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private static final double kRadPositionTolerance = Math.toRadians(4);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private boolean velocityControlmode;
    PIDController positionFeedBackController;
    ArmFeedforward positionFeedForwardController;

    PIDController velocityFeedBackController;
    ArmFeedforward velocityFeedForwardController;
    double desiredDegAngle = 0;
    public Pivot(PivotIO io)  {
        this.io = io;
        positionFeedBackController = new PIDController(3, 0, 0);
        positionFeedForwardController = new ArmFeedforward(0.0,-.4, 6); // 8
        positionFeedBackController.setTolerance(kRadPositionTolerance);

        velocityFeedBackController = new PIDController(0, 0, 0);
        velocityFeedForwardController = new ArmFeedforward(0, 0, 0);

        io.configureController(positionFeedForwardController, positionFeedBackController,
            velocityFeedForwardController, velocityFeedBackController);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("Pivot/desiredDegAngle", desiredDegAngle);
        io.setPosition(Math.toRadians(desiredDegAngle));
    }

    public void setDegAngle(double angle) {
        desiredDegAngle = angle;
    }

    public void setVelocityControlMode(boolean b){  
        velocityControlmode = b;
    };

    public void setVelocitySetPoint(double goalVelocityRadPerSec){
        io.setVelocity(goalVelocityRadPerSec);
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
    public void toggleMode() {
        io.toggleMode();
    }
}
