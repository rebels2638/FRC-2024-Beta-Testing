package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOFalcon extends SubsystemBase implements ElevatorIO {
    private static final double kMotorToOutputShaftRatio = 1/6.0; 
    private static final double kSproketDiameterMeters = 0.032;
    private static final double kFIRST_STAGE_TO_SECOND = 2.054054054054054;
    private static final double kSECOND_STAGE_TO_THIRD = 1.513157894736842;

    private TalonFX m_motor1 = new TalonFX(17);
    private TalonFX m_motor2 = new TalonFX(18);
    // private static final double kMAX_CURRENT_AMPS = 35; //Let the smart current handler in the motorControllers handle it. 
    private static final double kMAX_VOLTAGE = 12;
    private static final double kELEVATOR_ANGLE_SIN = Math.sin(Math.toRadians(23));
    
    private static final double kMIN_SHOOTER_HEIGHT = 0;
    private static final double kMAX_SHOOTER_HEIGHT= 0.52;

    private static final double kMIN_CLIMBER_HEIGHT = 0;
    private static final double kMAX_CLIMBER_HEIGHT= 0.7;

    private double kCLIMB_KG = 12;
    private PIDController positionFeedBackController = new PIDController(0, 0, 0);
    private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);
    private ElevatorFeedforward climbElevatorFeedforwardController = new ElevatorFeedforward(0,0,0);

    private double lastShooterHeightMeters;
    private double lastClimberHeightMeters;

    public ElevatorIOFalcon() {

        m_motor1.clearStickyFault_BootDuringEnable();
        m_motor2.clearStickyFault_BootDuringEnable();
        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.setNeutralMode(NeutralModeValue.Brake);
        // m_motor2.Follow(m_motor1); TODO: figure this out, DOESNT WORK LOL

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.shooterHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND;

        inputs.climberHeightMeters = m_motor1.getPosition().getValueAsDouble() * kMotorToOutputShaftRatio * Math.PI *
                                                            kSproketDiameterMeters * kFIRST_STAGE_TO_SECOND * kSECOND_STAGE_TO_THIRD * kELEVATOR_ANGLE_SIN;

        inputs.voltageOut = m_motor1.getDutyCycle().getValueAsDouble() * kMAX_VOLTAGE;

        inputs.reachedSetpoint = positionFeedBackController.atSetpoint();

        lastClimberHeightMeters = inputs.climberHeightMeters;
        lastShooterHeightMeters = inputs.shooterHeightMeters;
    }

    @Override
    // sould be called periodically
    // currentPositionMeters is in what ever elevator compunent (shooter/climber) you want to move
    public void setHeightMeters(double goalPositionMeters, boolean isShooterHeight, boolean isClimbing, boolean isRaw) {
        double currentPositionMeters;
        if (isShooterHeight) {
            currentPositionMeters = lastShooterHeightMeters;
        }
        else {
            currentPositionMeters = lastClimberHeightMeters;
        }
        if(isRaw){
            setVoltage(goalPositionMeters);
            return;
        }
    
        
        // cheking for over extension
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
            
            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
            double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);

            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            outVoltage = RebelUtil.constrain(outVoltage, -12, 12);

            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            m_motor1.setVoltage(outVoltage);
            m_motor2.setVoltage(outVoltage);
            return;
        }
        //Not Shooterheight and Not climbing. When will we ever use this. This is default case. But we should consider this at a later date.
        else if (!isShooterHeight && !isClimbing) {
            // here, our controllers are calibrated for the second stage, so we will just move the second stage to the apropriate position (two times lower) to set the third
            goalPositionMeters /= kSECOND_STAGE_TO_THIRD;
            currentPositionMeters /= kSECOND_STAGE_TO_THIRD; 

            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
            
            double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);
            
            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            outVoltage = RebelUtil.constrain(outVoltage, -12, 12);

            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            m_motor1.setVoltage(outVoltage);
            m_motor2.setVoltage(outVoltage);
            return;
        }
        //move climber down
        else {
            // here, our controllers are calibrated for the second stage, so we will just move the second stage to the apropriate position (two times lower) to set the third
            goalPositionMeters /= kSECOND_STAGE_TO_THIRD;
            currentPositionMeters /= kSECOND_STAGE_TO_THIRD; 
            
            positionFeedBackController.setSetpoint(goalPositionMeters);
            double feedBackControllerVoltage = positionFeedBackController.calculate(currentPositionMeters);
            double accel = feedBackControllerVoltage == 0 ? 0 : feedBackControllerVoltage < 0 ? -1: 1; //Changes direction of accel given the feedbackcontroller voltage.
            double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, accel);            
       
            
            double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            outVoltage = RebelUtil.constrain(outVoltage, -12, 12);

            Logger.recordOutput("Elevator/voltageOut", outVoltage);
            m_motor1.setVoltage(outVoltage);
            m_motor2.setVoltage(outVoltage);
        }
    } 

    public void setVoltage(double voltage){
        Logger.recordOutput("Elevator/voltageOut", voltage);
        m_motor1.setVoltage(voltage);
        m_motor2.setVoltage(voltage);
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb, ElevatorFeedforward Climbff,double kCLIMB_KG) {
        this.kCLIMB_KG = kCLIMB_KG;
        positionFeedBackController = pfb;
        positionFeedForwardController = pff;
        climbElevatorFeedforwardController = Climbff;
    }

    @Override
    public void zeroHeight() {
        m_motor1.setPosition(0.0);
        m_motor2.setPosition(0.0);
    }

}