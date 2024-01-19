package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIONeo extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);
    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward velocityFeedForwardController = new ElevatorFeedforward(0, 0, 0);

    public ElevatorIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults();
        m_motor.setInverted(true);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRad = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.positionDeg = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * 360;

        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.velocityDegSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * 360;
    }

    @Override
    // sould be called periodically
    public void setHightMeters(double goalPositionRad, double currentRadAngle) {
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionRad, 0);
        
        positionFeedBackController.setSetpoint(goalPositionRad);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentRadAngle);
        System.out.println("CALLED");
        Logger.recordOutput("Elevator/voltageOut", feedForwardVoltage + feedBackControllerVoltage);
        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    @Override
    // sould be called periodically
    public void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, 0);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);
        Logger.recordOutput("Elevator/voltageOut", feedForwardVoltage + feedBackControllerVoltage);
        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);

    } 

    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb, 
                                          ElevatorFeedforward vff, PIDController vfb ) {
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    @Override
    public boolean reachedSetpoint(boolean isPositionalControl) {
        if (isPositionalControl) {
            return positionFeedBackController.atSetpoint();
        } else{
        return velocityFeedBackController.atSetpoint();
        }
    }

    @Override
    public void zeroHight() {
        m_motor.getEncoder().setPosition(0.0);
    }

}