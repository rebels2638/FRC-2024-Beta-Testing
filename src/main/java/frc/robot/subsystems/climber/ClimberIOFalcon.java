package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOFalcon extends SubsystemBase implements ClimberIO {
    private static final double kMotorToOutputShaftRatio = 1/80.0; 
    private static final double kSproketDiameterMeters = 0.0126903553299492;

    // dont know device ID
    private TalonFX m_motor1 = new TalonFX(18); 
    private TalonFX m_motor2 = new TalonFX(19);
    private static final double kMAX_CURRENT_AMPS = 35;
    private static final double kMAX_VOLTAGE = 12;

    private static final double kMIN_CLIMBER_HEIGHT = 0;
    private static final double kMAX_CLIMBER_HEIGHT= 12;

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);

    private double climberHeightMeters;
    private double goalPositionMeters;

    public ClimberIOFalcon() {
        m_motor1.setInverted(true);
        m_motor2.setInverted(true);
        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.setNeutralMode(NeutralModeValue.Brake);        

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters;
        inputs.voltageOut = m_motor1.getDutyCycle().getValueAsDouble() * kMAX_VOLTAGE;

        climberHeightMeters = inputs.climberHeightMeters;

        inputs.reachedSetpoint = positionFeedBackController.atSetpoint();
    }

    @Override
    // sould be called periodically
    // currentPositionMeters is in what ever elevator compunent (shooter/climber) you want to move
    public void setHeightMeters(double goalPositionMeters) {
        this.goalPositionMeters = goalPositionMeters;
        double currentPositionMeters = climberHeightMeters;
        
        positionFeedBackController.setSetpoint(goalPositionMeters);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
        double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);

        double voltageOut = feedForwardVoltage + feedBackControllerVoltage;
        voltageOut = RebelUtil.constrain(voltageOut, -12, 12);

        if ((currentPositionMeters > kMAX_CLIMBER_HEIGHT && voltageOut > 0) || 
            (currentPositionMeters < kMIN_CLIMBER_HEIGHT && voltageOut < 0) || 
            (goalPositionMeters > kMAX_CLIMBER_HEIGHT || goalPositionMeters < kMIN_CLIMBER_HEIGHT)) {
            voltageOut = 0;
        }
        
        Logger.recordOutput("Climber/voltageOut", voltageOut);
        m_motor1.setVoltage(voltageOut);
        m_motor2.setVoltage(voltageOut);
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
    public void zeroHeight() {
        m_motor1.setPosition(0.0);
        m_motor2.setPosition(0);
    }

}