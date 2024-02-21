package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

public class PivotIONeo extends SubsystemBase implements PivotIO {
    private static final double kMotorToOutputShaftRatio = 0.00625;
    private CANSparkMax m_motor = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless); //Motor IDs DONE 2/8/2024

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward positionFeedForwardController = new ArmFeedforward(0, 0, 0);
    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private ArmFeedforward velocityFeedForwardController = new ArmFeedforward(0, 0, 0);
    private static final double kMAX_POSITION_RAD = Math.toRadians(98);
    private static final double kMIN_POSITION_RAD = Math.toRadians(-1);
    private static final double kMAX_VOLTAGE = 12;
    private static double currentRadAngle;
    private static double currentVelocityRadPerSec;

    public PivotIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake); //This is desirable, trust me kBrake
        m_motor.clearFaults();
        m_motor.setInverted(true);
    }

    
    public void toggleMode() {
        if (this.m_motor.getIdleMode() == CANSparkMax.IdleMode.kBrake) {
            this.m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);   
        }
        else {
            this.m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2;
        currentRadAngle = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.positionDeg = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * 360;

        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2;
        currentVelocityRadPerSec =  m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2;
        inputs.velocityDegSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * 360;

        inputs.reachedSetpoint = reachedSetpoint(true);
    }
    @Override
    // sould be called periodically
    public void setPosition(double goalPositionRad) {

        double ffVelo = 0;
        if (goalPositionRad > currentRadAngle) {
            ffVelo = 1;
        }
        else if (goalPositionRad < currentRadAngle) {
            ffVelo = -1;
        }
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionRad, ffVelo);

        if (Math.abs(currentRadAngle - goalPositionRad) < positionFeedBackController.getPositionTolerance()) {
            feedForwardVoltage = 0;
        }
        Logger.recordOutput("Pivot/feedForwardVoltage", feedForwardVoltage);

        positionFeedBackController.setSetpoint(goalPositionRad);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentRadAngle);
        double voltageOut = feedForwardVoltage + feedBackControllerVoltage;
        
        voltageOut = RebelUtil.constrain(voltageOut, -12, 12);

        if ((currentRadAngle > kMAX_POSITION_RAD && voltageOut > 0) || 
                (currentRadAngle < kMIN_POSITION_RAD && voltageOut < 0) || 
                (goalPositionRad > kMAX_POSITION_RAD || goalPositionRad < kMIN_POSITION_RAD)) {
                    voltageOut = 0;
        }

        Logger.recordOutput("Pivot/voltageOut", voltageOut);
        m_motor.setVoltage(voltageOut);
    } 

    @Override
    // sould be called periodically
    public void setVelocity(double goalVelocityRadPerSec) {
        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, 0);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);
        double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
        if (outVoltage > kMAX_VOLTAGE) {
            outVoltage = 12;
        }
        else if (outVoltage < -kMAX_VOLTAGE) {
            outVoltage = -12;
        }
        //Logger.recordOutput("Pivot/voltageOut", outVoltage);
        m_motor.setVoltage(outVoltage);

    } 

    public void setVoltage(double voltage){
        if ((currentRadAngle > kMAX_POSITION_RAD && voltage > 0)|| (currentRadAngle < kMIN_POSITION_RAD && voltage < 0)) {
            return;
        }
        else{
            Logger.recordOutput("Pivot/voltageOut", voltage);
            m_motor.setVoltage(voltage);
        }
    }

    @Override
    public void configureController(ArmFeedforward pff, PIDController pfb, 
                                        ArmFeedforward vff, PIDController vfb ) {
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    private boolean reachedSetpoint(boolean isPositionalControl) {
        if (isPositionalControl) {
            return positionFeedBackController.atSetpoint();
        } else{
        return velocityFeedBackController.atSetpoint();
        }
    }

    @Override
    public void zeroAngle() {
        m_motor.getEncoder().setPosition(0.0);
    }

}