// package frc.robot.subsystems.elevator;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ElevatorIOSim extends SubsystemBase implements ElevatorIO {

//     private double desiredHightMeters = 0;

//     @Override
//     public void updateInputs(ElevatorIOInputs inputs) {
//         inputs.hightMeters = desiredHightMeters;
//     }

//     public abstract void updateInputs(ElevatorIOInputs inputs);

//     public abstract void setHightMeters(double goalPositionMeters, double currentHightMeters, 
//                                             boolean isShooterHight, boolean isClimbing);

//     public abstract void configureController(ElevatorFeedforward pff, PIDController pfb, double kCLIMB_KG);

//     public abstract void setVoltage(double voltage);

//     public abstract boolean reachedSetpoint();

//     public abstract void zeroHeight();
// }