package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIONeo extends SubsystemBase implements ShooterIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor1 = new CANSparkMax(21, MotorType.kBrushless); 
    private CANSparkMax m_motor2 = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);
    
    private static final double kMAX_VOLTAGE = 12;

    public ShooterIONeo() {
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor1.clearFaults();
        m_motor1.setInverted(true);

        m_motor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor2.clearFaults();
        m_motor2.setInverted(true);
        m_motor2.follow(m_motor1);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = m_motor1.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
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
        Logger.recordOutput("Shooter/voltageOut", outVoltage);
        m_motor1.setVoltage(feedForwardVoltage + feedBackControllerVoltage);

    } 

    @Override
    public void setVoltage(double voltage){
        m_motor1.setVoltage(voltage);
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

}