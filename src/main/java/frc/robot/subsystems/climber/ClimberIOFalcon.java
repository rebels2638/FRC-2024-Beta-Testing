package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOFalcon extends SubsystemBase implements ClimberIO {
    private static final double kMotorToOutputShaftRatio = 1/80.0; 
    private static final double kSproketDiameterMeters = 0.0508;

    ShuffleboardTab tab;

    // dont know device ID
    private TalonFX m_motor1 = new TalonFX(21); // 19
    private TalonFX m_motor2 = new TalonFX(20); // dont change
    private static final double kMAX_CURRENT_AMPS = 35;
    private static final double kMAX_VOLTAGE = 12;

    private static final double kMIN_CLIMBER_HEIGHT = -2; // 0
    private static final double kMAX_CLIMBER_HEIGHT = 0.34; // TODO: Alt. for man. 0.35 max as of 3/20/2024, .36

    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);

    private double climberHeightMeters = 0;
    private double goalPositionMeters;
    double currentPositionMeters = climberHeightMeters;
    double currentPositionMeters2 = climberHeightMeters;

 
    public ClimberIOFalcon() {
        m_motor1.setInverted(true);
        m_motor2.setInverted(false);
        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.setNeutralMode(NeutralModeValue.Brake); 

        m_motor1.clearStickyFaults();
        m_motor2.clearStickyFaults();

        // m_motor1.optimizeBusUtilization();
        // m_motor2.optimizeBusUtilization();


        
        climberHeightMeters = 0; //initial zero at current position

        zeroHeight();

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        //motor1
        inputs.climberHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters;
        inputs.voltageOut = m_motor1.getDutyCycle().getValueAsDouble() * kMAX_VOLTAGE;

        climberHeightMeters = inputs.climberHeightMeters;
        currentPositionMeters = climberHeightMeters;

        currentPositionMeters2 = m_motor2.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters;
        inputs.reachedSetpoint = positionFeedBackController.atSetpoint();
        inputs.goalPositionMeters = this.goalPositionMeters;    

        // System.out.println("ClimberHeightMeters : " + climberHeightMeters);
        // System.out.println("ClimberHeightMeters2: " + m_motor2.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters);

    }

    @Override
    // should be called periodically
    // currentPositionMeters is in what ever elevator component (shooter/climber) you want to move
    public void setHeightMeters(double goalPositionMeters) {
        this.goalPositionMeters = goalPositionMeters;
        double currentPositionMeters = climberHeightMeters;
        
        //Motor1
        positionFeedBackController.setSetpoint(goalPositionMeters);
        double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
        double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
        double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);

        //Motor2
        double feedBackControllerVoltage2 = positionFeedBackController.calculate(currentPositionMeters2);
        double accel2 = feedBackControllerVoltage2 == 0 ? 0 : feedBackControllerVoltage2 < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
        double feedForwardVoltage2 = positionFeedForwardController.calculate(goalPositionMeters, accel2);

        double voltageOut = feedForwardVoltage + feedBackControllerVoltage;
        voltageOut = RebelUtil.constrain(voltageOut, -12, 12);

        double voltageOut2 = feedBackControllerVoltage2 + feedForwardVoltage2;
        voltageOut2 = RebelUtil.constrain(voltageOut, -12, 12);


        if ((currentPositionMeters > kMAX_CLIMBER_HEIGHT && voltageOut > 0) || 
            (currentPositionMeters < kMIN_CLIMBER_HEIGHT && voltageOut < 0) || 
            (goalPositionMeters > kMAX_CLIMBER_HEIGHT || goalPositionMeters < kMIN_CLIMBER_HEIGHT)) {
            voltageOut = 0;
        }
        if ((currentPositionMeters2> kMAX_CLIMBER_HEIGHT && voltageOut > 0) || 
            (currentPositionMeters2 < kMIN_CLIMBER_HEIGHT && voltageOut < 0) || 
            (goalPositionMeters > kMAX_CLIMBER_HEIGHT || goalPositionMeters < kMIN_CLIMBER_HEIGHT)) {
            voltageOut2 = 0;
        }
        // // Logger.recordOutput("Climber/voltageOut", voltageOut);
        m_motor1.setVoltage(voltageOut);
        m_motor2.setVoltage(voltageOut2);
    }

    public double getHeightMeters() {
        return this.climberHeightMeters;
    }

    public void setVoltage(double voltage){
        if ((climberHeightMeters >= kMAX_CLIMBER_HEIGHT && voltage > 0) || 
            (climberHeightMeters <= kMIN_CLIMBER_HEIGHT && voltage < 0)) {
            voltage = 0;
        }

    //    Logger.recordOutput("Climber/voltageOut", voltage);

        m_motor1.setVoltage(voltage);
        m_motor2.setVoltage(voltage);
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb) {
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
    }

    @Override
    public void zeroHeight() {
        m_motor1.setPosition(0);
        m_motor2.setPosition(0);
    }

}