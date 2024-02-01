package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIONeo extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 1/6.0; // 6
    private static final double kSproketDiameterMeters = 0.035;
    private static final double kFIRST_STAGE_TO_SECOND = 2;
    private CANSparkMax m_motor1 = new CANSparkMax(15, MotorType.kBrushless); 
    private CANSparkMax m_motor2 = new CANSparkMax(16, MotorType.kBrushless);
    private static final double kMAX_CURRENT_AMPS = 35;
    private static final double kMAX_VOLTAGE = 12;
    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);

    public ElevatorIONeo() {
        m_motor1.setInverted(true);
        m_motor2.setInverted(true);
        m_motor1.clearFaults();
        m_motor2.clearFaults();
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor2.follow(m_motor1);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.hightMeters = m_motor1.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND;
        inputs.voltageOut = m_motor1.getAppliedOutput() * kMAX_VOLTAGE;
    }

    @Override
    // sould be called periodically
    public void setHightMeters(double goalPositionMeters, double currentPositionMeters) {
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters - currentPositionMeters, 0);
        
        positionFeedBackController.setSetpoint(goalPositionMeters);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
        
        Logger.recordOutput("Elevator/voltageOut", feedForwardVoltage + feedBackControllerVoltage);
        if (feedForwardVoltage + feedBackControllerVoltage > kMAX_VOLTAGE) {
            return;
        }
        m_motor1.setVoltage(feedForwardVoltage + feedBackControllerVoltage);
    } 

    public void setVoltage(double voltage){
        m_motor1.setVoltage(voltage);
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
        m_motor1.getEncoder().setPosition(0.0);
        m_motor2.getEncoder().setPosition(0);
    }

}