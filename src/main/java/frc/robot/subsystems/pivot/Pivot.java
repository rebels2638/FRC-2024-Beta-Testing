package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private static final double kRadPositionTolerance = Math.toRadians(5);

    private static PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private boolean velocityControlmode;

    PIDController upPositionFeedBackController; 
    PIDController downPositionFeedBackController;

    ArmFeedforward downPositionFeedForwardController;
    ArmFeedforward upPositionFeedForwardController;

    private static Pivot instance = null;

    PIDController velocityFeedBackController;
    ArmFeedforward velocityFeedForwardController;
    double desiredDegAngle = 0;
    public Pivot(PivotIO io)  {
        Pivot.io = io;
        upPositionFeedBackController = new PIDController(4, 0, 0.03); //0.04
        upPositionFeedBackController.setTolerance(kRadPositionTolerance);
        upPositionFeedForwardController = new ArmFeedforward(0.0,0,5); // 0,0,76

        downPositionFeedBackController = new PIDController(7, 0, 0);
        downPositionFeedBackController.setTolerance(kRadPositionTolerance);
        downPositionFeedForwardController = new ArmFeedforward(0,0, 3); // 8

        io.configureController(downPositionFeedForwardController, downPositionFeedBackController);
    }

    @Override
    public void periodic() {
        if (desiredDegAngle > 45) {
            io.configureController(downPositionFeedForwardController, downPositionFeedBackController);
        }
        else if (desiredDegAngle < 45) {
            io.configureController(upPositionFeedForwardController, upPositionFeedBackController);
        }
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("Pivot/desiredDegAngle", desiredDegAngle);
        io.setPosition(Math.toRadians(desiredDegAngle));
        // io.setPosition(desiredDegAngle);
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
    public void TorusAngleReset(){
        io.TorusAngleReset();
    }
    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }
    public void toggleMode() {
        io.toggleMode();
    }

    public static Pivot getInstance(){
        if ( Pivot.instance == null){
            return new Pivot(Pivot.io);
        }
        return instance;
    }
    public static Pivot setInstance(Pivot inst){
        Pivot.instance = inst;
        return Pivot.instance;
    }
}
