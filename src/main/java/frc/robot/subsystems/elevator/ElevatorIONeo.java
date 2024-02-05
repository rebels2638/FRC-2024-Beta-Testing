package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

public class ElevatorIONeo extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 1/6.0; 
    private static final double kSproketDiameterMeters = 0.035;
    private static final double kFIRST_STAGE_TO_SECOND = 2;
    private static final double kSECOND_STAGE_TO_THIRD = 2;
    private CANSparkMax m_motor1 = new CANSparkMax(15, CANSparkMax.MotorType.kBrushless); 
    private CANSparkMax m_motor2 = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless);
    private static final double kMAX_CURRENT_AMPS = 35;
    private static final double kMAX_VOLTAGE = 12;
    private static final double kELEVATOR_ANGLE_COS = Math.cos(Math.toRadians(23));
    
    private static final double kMIN_SHOOTER_HEIGHT = 0;
    private static final double kMAX_SHOOTER_HEIGHT= 12;

    private static final double kMIN_CLIMBER_HEIGHT = 0;
    private static final double kMAX_CLIMBER_HEIGHT= 12;

    private double kCLIMB_KG = 12;
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
        inputs.shooterHeightMeters = m_motor1.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND;

        inputs.climberHeightMeters = m_motor1.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI *
                                                            kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND * kSECOND_STAGE_TO_THIRD * kELEVATOR_ANGLE_COS;
        inputs.voltageOut = m_motor1.getAppliedOutput() * kMAX_VOLTAGE;
    }

    @Override
    // sould be called periodically
    // currentPositionMeters is in what ever elevator compunent (shooter/climber) you want to move
    public void setHeightMeters(double goalPositionMeters, double currentPositionMeters, boolean isShooterHeight, boolean isClimbing) {
        if (isShooterHeight) {
            if (currentPositionMeters > kMAX_SHOOTER_HEIGHT || currentPositionMeters < kMIN_SHOOTER_HEIGHT || 
                goalPositionMeters > kMAX_SHOOTER_HEIGHT || goalPositionMeters < kMIN_SHOOTER_HEIGHT) {
                    return;
            }
        }
        else {
            if (currentPositionMeters > kMAX_CLIMBER_HEIGHT || currentPositionMeters < kMIN_CLIMBER_HEIGHT || 
                goalPositionMeters > kMAX_CLIMBER_HEIGHT || goalPositionMeters < kMIN_CLIMBER_HEIGHT) {
                    return;
            }
        }
        
        if (isShooterHeight) {
            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters - currentPositionMeters, 0);
            
            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
            
            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            
            if (outVoltage > kMAX_VOLTAGE) {
                outVoltage = 12;
            }
            else if (outVoltage < -kMAX_VOLTAGE) {
                outVoltage = -12;
            }
            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            
            m_motor1.setVoltage(outVoltage);
            return;
        }
        // just move the climber up
        else if (!isShooterHeight && !isClimbing) {
            // here, our controllers are calibrated for the second stage, so we will just move the second stage to the apropriate position (two times lower) to set the third
            goalPositionMeters /= kSECOND_STAGE_TO_THIRD;
            currentPositionMeters /= kSECOND_STAGE_TO_THIRD; 

            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters - currentPositionMeters, 0);
            
            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
            
            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            if (outVoltage > kMAX_VOLTAGE) {
                outVoltage = 12;
            }
            else if (outVoltage < -kMAX_VOLTAGE) {
                outVoltage = -12;
            }
            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            m_motor1.setVoltage(outVoltage);
            return;
        }
        // move climber down
        else {
            // here, our controllers are calibrated for the second stage, so we will just move the second stage to the apropriate position (two times lower) to set the third
            goalPositionMeters /= kSECOND_STAGE_TO_THIRD;
            currentPositionMeters /= kSECOND_STAGE_TO_THIRD; 
            
            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters - currentPositionMeters, 0);
            
            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters) + kCLIMB_KG;
            
            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            if (outVoltage > kMAX_VOLTAGE) {
                outVoltage = 12;
            }
            else if (outVoltage < -kMAX_VOLTAGE) {
                outVoltage = -12;
            }
            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            m_motor1.setVoltage(outVoltage);
        }
        
    } 

    public void setVoltage(double voltage){
        m_motor1.setVoltage(voltage);
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb, double kCLIMB_KG) {
        this.kCLIMB_KG = kCLIMB_KG;
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