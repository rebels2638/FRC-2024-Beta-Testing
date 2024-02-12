// package frc.robot.subsystems.climber;

// import org.littletonrobotics.junction.AutoLog;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;

// public interface ClimberIO {
//     @AutoLog
//     public static class ClimberIOInputs {
//         public double climberHeightMeters;
//         public double voltageOut;
//         public boolean reachedSetpoint; 
//     }

//     public default void updateInputs(ClimberIOInputs inputs) {}

//     public default void setHeightMeters(double goalPositionMeters, boolean isClimbing) {}

//     public default void configureController(ElevatorFeedforward pff, PIDController pfb, double kCLIMB_KG) {}

//     public default void setVoltage(double voltage) {}
    
//     public default void zeroHeight() {}
    
// }
