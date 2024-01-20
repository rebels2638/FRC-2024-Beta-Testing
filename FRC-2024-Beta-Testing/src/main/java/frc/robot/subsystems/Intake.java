package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static final double kMaxSpeed = 5.0; // rotations per second 
    public static final double kMaxAcceleration = 2.2; // rotations per second squared 

    // private static final double kWheelRadius = 0.0; // meters
    private static final int kEncoderResolution = 42; 
    private static final int kGearingRatio = 4;

    private static final double kNativeUnitsPerRotation = kEncoderResolution * kGearingRatio;
    private static final double kRotationsPerNativeUnit = 1 / kNativeUnitsPerRotation;
    // private static  final double kMetersPerRotation = 2 * Math.PI * kWheelRadius;
    // private static final double kRotationsPerMeter = 1 / kMetersPerRotation;

    private final CANSparkMax m_motor = new CANSparkMax(1, MotorType.kBrushless); // ID NUMBER

    private final PIDController m_controller = new PIDController(5.0, 0, 0); // IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD 
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 3.0, 0); // IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA

    private double m_velocitySetpoint = 0;
    private double m_voltageSetpoint = 0;

    private double m_lastVelocity = 0;
    private double m_lastTime = Timer.getFPGATimestamp();


    public Intake() {
        m_motor.setInverted(false); 
        m_motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        setVelocitySetpoint(0);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }


    public double rotationsToNative(double rotations) {
        return rotations * kNativeUnitsPerRotation;
    }

    public double nativeToRotations(double encoderUnits) {
        
        return encoderUnits * kRotationsPerNativeUnit;
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        m_velocitySetpoint = velocitySetpoint;
    }

    public double getCurrentVelocity() {
        return m_motor.getEncoder().getVelocity() / 60; // getVelocity() returns in RPM
    }

    public double getCurrentAcceleration() {
        return (getCurrentVelocity() - m_lastVelocity) / (Timer.getFPGATimestamp() - m_lastTime);
    }

    public double getVelocitySetpoint() {
        return m_velocitySetpoint;
    }

    @Override
    public void periodic() {
        double feedforward = m_feedforward.calculate(getVelocitySetpoint());
        double pid = m_controller.calculate(getCurrentVelocity(), getVelocitySetpoint());
        double voltage = RebelUtil.constrain(feedforward + pid, -12, 12);

        m_voltageSetpoint = voltage;
        m_motor.setVoltage(m_voltageSetpoint);

        m_lastVelocity = getCurrentVelocity();
        m_lastTime = Timer.getFPGATimestamp();
    }

    public void breakMotor() {
        m_motor.stopMotor();
    }
}


