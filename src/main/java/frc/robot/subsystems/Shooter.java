// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.lib.input.RebelUtil;
// import frc.robot.Robot;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.Timer;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;

// public class Shooter extends SubsystemBase {
//     private static Shooter instance = null;

//     public static final double kMaxSpeed = 1.5; // meters per second 
//     public static final double kMaxAcceleration = 2.2; // meters per second squared 

//     private static final double kWheelRadius = 0.018191; // meters
//     private static final int kEncoderResolution = 2048; 
//     private static final int kGearingRatio = 6;

//     private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
//     private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
//     private static  final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
//     private static final double kRotationsPerMeter = 1 / kMetersPerRotation;
    
//     private final WPI_TalonFX m_motor1 = new WPI_TalonFX(0);
//     private final WPI_TalonFX m_motor2 = new WPI_TalonFX(3);

//     private final PIDController m_controller = new PIDController(5.0, 0, 0); // ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD 
//     private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 3.0, 0); // ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA

//     private double m_velocitySetpoint = 0;
//     private double m_voltageSetpoint = 0;

//     private double m_lastVelocitySetpoint = 0;
//     private double m_lastVelocity = 0;
//     private double m_lastTime = Timer.getFPGATimestamp();

//     private static double kUpperLimit = 0.71;
//     private static double kLowerLimit = -0.02; 

//     public Shooter() {
//         m_motor1.setInverted(false); 
//         m_motor2.setInverted(false);

//         m_motor1.setNeutralMode(NeutralMode.Brake);
//         m_motor2.setNeutralMode(NeutralMode.Brake);

//         m_motor1.set(ControlMode.PercentOutput, 0);
//         m_motor2.set(ControlMode.PercentOutput, 0);

//         setVelocitySetpoint(0);
//     }

//     public static Shooter getInstance() {
//         if (instance == null) {
//             instance = new Shooter();
//         }
//         return instance;
//     }


//     /*
//     * Convert from TalonFX flywheel position in meters to native units and vice versa
//     */
//     public double heightToNative(double heightUnits) {
//         return heightUnits * kRotationsPerMeter * kNativeUnitsPerRotation;
//     }

//     public double nativeToHeight(double encoderUnits) {
        
//         return encoderUnits * kRotationsPerNativeUnit * kMetersPerRotation;
//     }

//     public void setVelocitySetpoint(double velocitySetpoint) {
//         m_velocitySetpoint = velocitySetpoint;
//     }

//     public double getCurrentEncoderRate() {
//         return m_motor1.getSensorCollection().getIntegratedSensorVelocity() * 10; // motor velocity is in ticks per 100ms
//     }

//     public double getCurrentVelocity() {
//         return nativeToHeight(getCurrentEncoderRate());
//     }

//     public double getCurrentAcceleration() {
//         return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
//     }

//     public double getVelocitySetpoint() {
//         return m_velocitySetpoint;
//     }

//     @Override
//     public void periodic() {
//         double feedforward = m_feedforward.calculate(getVelocitySetpoint());
//         double pid = m_controller.calculate(getCurrentVelocity(), getVelocitySetpoint());
//         double voltage = RebelUtil.constrain(feedforward + pid, -12.0, 12.0);

//         m_voltageSetpoint = voltage;
//         m_motor1.setVoltage(m_voltageSetpoint);
//         m_motor2.setVoltage(m_voltageSetpoint);


//         m_lastVelocitySetpoint = getVelocitySetpoint();
//         m_lastVelocity = getCurrentVelocity();
//         m_lastTime = Timer.getFPGATimestamp();
//     }

//     public void breakMotor() {
//         m_motor1.stopMotor();
//         m_motor2.stopMotor();
//     }
// }

