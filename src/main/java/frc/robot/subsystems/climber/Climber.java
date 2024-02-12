// package frc.robot.subsystems.climber;

// import org.littletonrobotics.junction.Logger;


// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase{

//     private final ClimberIO io;
//     private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
//     PIDController positionFeedBackController;
//     ElevatorFeedforward positionFeedForwardController;

//     // PIDController velocityFeedBackController;
//     // ElevatorFeedforward velocityFeedForwardController; //Literally never gonna be used
//     private static final double kPID_TOLERANCE_METERS = 0.01; //this is 1cm 
//     private static final double kCLIMB_KG = 12;

//     public Climber(ClimberIO io)  {
//         this.io = io;
//         positionFeedBackController = new PIDController(6, 0, 0); // 0 0 0 
//         //
//         positionFeedForwardController = new ElevatorFeedforward(0.33, 0.14, 0); //0.008 0.31 31

        
//         // velocityFeedBackController = new PIDController(0, 0, 0);
//         // velocityFeedForwardController = new ElevatorFeedforward(0,0, 0);
        
//         positionFeedBackController.setTolerance(kPID_TOLERANCE_METERS);

//         io.configureController(positionFeedForwardController, positionFeedBackController, kCLIMB_KG);
//     }

//     @Override
//     public void periodic() {
//         io.updateInputs(inputs);
//         Logger.processInputs("Elevator", inputs);
//     }

//     public void setHeightMeters(double goalPositionMeters,boolean isClimbing) {
//         Logger.recordOutput("Climber/desiredClimberHeight");
//         io.setHeightMeters(goalPositionMeters, isClimbing);

//         return;
//     }

//     public void setVoltage(double voltage){
//         io.setVoltage(voltage);
//         return;
//     }

//     public double getClimberHeightMeters() {
//         return inputs.climberHeightMeters;
//     }

//     public void zeroHeight() {
//         io.zeroHeight();
//     }
//     public boolean reachedSetpoint() {
//         return inputs.reachedSetpoint;
//     }
// }