package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIONeo extends SubsystemBase implements ShooterIO {
    private static final double kMotorToOutputShaftRatio = 1; // 1:1 rn
    private static final double kSproketDiameterMeters = 1; // doesn't exist
    private CANSparkMax m_motor1 = new CANSparkMax(21, MotorType.kBrushless); 
    private CANSparkMax m_motor2 = new CANSparkMax(22, MotorType.kBrushless);

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    public ShooterIONeo() {
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor1.clearFaults();
        m_motor1.setInverted(false);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.hightMeters = m_motor.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * 2 * kSproketDiameterMeters;
    }

    @Override
    // sould be called periodically
    public void setHightMeters(double goalPositionMeters, double currentPositionMeters) {
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, 0);
        
        positionFeedBackController.setSetpoint(goalPositionMeters);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
        
        System.out.println("CALLED");
        Logger.recordOutput("Elevator/voltageOut", feedForwardVoltage + feedBackControllerVoltage);
        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb) {
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
    }

    @Override
    public boolean reachedSetpoint() {
        return positionFeedBackController.atSetpoint();
    }

    @Override
    public void zeroHeight() {
        m_motor.getEncoder().setPosition(0.0);
    }

}