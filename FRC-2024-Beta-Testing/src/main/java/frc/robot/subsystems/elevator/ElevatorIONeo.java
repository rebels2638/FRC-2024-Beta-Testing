package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIONeo extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private static final double kSproketDiameterMeters = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);

    public ElevatorIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults();
        m_motor.setInverted(true);
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
    public void zeroHight() {
        m_motor.getEncoder().setPosition(0.0);
    }

}