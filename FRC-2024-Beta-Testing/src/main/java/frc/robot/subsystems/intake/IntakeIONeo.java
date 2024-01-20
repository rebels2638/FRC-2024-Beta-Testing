package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIONeo extends SubsystemBase implements IntakeIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    public IntakeIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults();
        m_motor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, 0);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);
        Logger.recordOutput("Pivot/voltageOut", feedForwardVoltage + feedBackControllerVoltage);
        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);

    } 

    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    @Override
    public boolean reachedSetpoint() {
        return velocityFeedBackController.atSetpoint();
    }

    @Override
    public boolean inIntake() {
        // TODO: WRITE THE BEAM BRAKE CODE
        return true;
    }

}