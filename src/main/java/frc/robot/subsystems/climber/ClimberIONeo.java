// package frc.robot.subsystems.climber;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.CANSparkMax;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Utils.RebelUtil;
// import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

// public class ClimberIONeo extends SubsystemBase implements ClimberIO {
//     private static final double kMotorToOutputShaftRatio = 1/6.0; 
//     private static final double kSproketDiameterMeters = 0.035;
//     private CANSparkMax m_motor1 = new CANSparkMax(18, CANSparkMax.MotorType.kBrushless); 
//     private CANSparkMax m_motor2 = new CANSparkMax(19, CANSparkMax.MotorType.kBrushless);
//     private static final double kMAX_CURRENT_AMPS = 35;
//     private static final double kMAX_VOLTAGE = 12;

//     private static final double kMIN_CLIMBER_HEIGHT = 0;
//     private static final double kMAX_CLIMBER_HEIGHT= 12;

//     private double kCLIMB_KG = 12;
//     private PIDController positionFeedBackController = new PIDController(0, 0, 0);
//     private ElevatorFeedforward positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);

//     private double climberHeightMeters;

//     public ClimberIONeo() {
//         m_motor1.setInverted(true);
//         m_motor2.setInverted(true);
//         m_motor1.clearFaults();
//         m_motor2.clearFaults();
//         m_motor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
//         m_motor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
//         m_motor2.follow(m_motor1);

//     }

//     @Override
//     public void updateInputs(ClimberIOInputs inputs) {
//         inputs.climberHeightMeters = m_motor1.getEncoder().getPosition() * kMotorToOutputShaftRatio * Math.PI * kSproketDiameterMeters;
//         inputs.voltageOut = m_motor1.getAppliedOutput() * kMAX_VOLTAGE;

//         climberHeightMeters = inputs.climberHeightMeters;

//         inputs.reachedSetpoint = positionFeedBackController.atSetpoint();
//     }

//     @Override
//     // sould be called periodically
//     // currentPositionMeters is in what ever elevator compunent (shooter/climber) you want to move
//     public void setHeightMeters(double goalPositionMeters, boolean isClimbing) {
//         // cheking for over extension
//         if (climberHeightMeters > kMAX_CLIMBER_HEIGHT || climberHeightMeters < kMIN_CLIMBER_HEIGHT || 
//             goalPositionMeters > kMAX_CLIMBER_HEIGHT || goalPositionMeters < kMIN_CLIMBER_HEIGHT) {
//                 return;
//         }
//         if (isClimbing) {
//             double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, 0);
            
//             positionFeedBackController.setSetpoint(goalPositionMeters);
//             double feedBackControllerVoltage = positionFeedBackController.calculate(climberHeightMeters) + kCLIMB_KG;
            
//             double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            
//             if (outVoltage > kMAX_VOLTAGE) {
//                 outVoltage = 12;
//             }
//             else if (outVoltage < -kMAX_VOLTAGE) {
//                 outVoltage = -12;
//             }
//             Logger.recordOutput("Elevator/voltageOut", outVoltage);
            
//             m_motor1.setVoltage(outVoltage);
//             return;
//         }

//         else {      
//             double feedForwardVoltage = positionFeedForwardController.calculate(goalPositionMeters, 0);
            
//             positionFeedBackController.setSetpoint(goalPositionMeters);
//             double feedBackControllerVoltage = positionFeedBackController.calculate(climberHeightMeters);
            
//             double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
//             if (outVoltage > kMAX_VOLTAGE) {
//                 outVoltage = 12;
//             }
//             else if (outVoltage < -kMAX_VOLTAGE) {
//                 outVoltage = -12;
//             }
//             Logger.recordOutput("Climber/voltageOut", outVoltage);
//             m_motor1.setVoltage(outVoltage);
//         }
        
//     } 

//     public void setVoltage(double voltage){
//         m_motor1.setVoltage(voltage);
//     }

//     @Override
//     public void configureController(ElevatorFeedforward pff, PIDController pfb, double kCLIMB_KG) {
//         this.kCLIMB_KG = kCLIMB_KG;
//         positionFeedBackController = pfb;
//         positionFeedForwardController = pff;
//     }

//     @Override
//     public void zeroHeight() {
//         m_motor1.getEncoder().setPosition(0.0);
//         m_motor2.getEncoder().setPosition(0);
//     }

// }