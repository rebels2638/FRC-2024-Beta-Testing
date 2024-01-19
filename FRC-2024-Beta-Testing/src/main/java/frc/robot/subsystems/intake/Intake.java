package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private CANSparkMax m_motor = new CANSparkMax(19, MotorType.kBrushless); 

    private static final double kMotorToOutputShaftRatio = 0.25;
    private static final double kPulsePerRotation = 4096;//rev hub reports 4096, without, it is reported as 42

    private static final double kSpikeAMPS = 10;


    private final PIDController feedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward feedForwardController = new SimpleMotorFeedforward(1.2, 0.03, 0);

    private double goalVelocityRadSec = 0;

    private static final int currentLimit = 20;
    private static final double voltageLimit = 12;

    public Intake() {
        m_motor.setSmartCurrentLimit(currentLimit);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/measuredVelocityRadSec", getVelocityRadSec());
        double feedForwardVoltage = feedForwardController.calculate(getVelocityRadSec(), goalVelocityRadSec);
        feedBackController.setSetpoint(goalVelocityRadSec);
        double feedBackControllerVoltage = feedBackController.calculate(getVelocityRadSec());
        double output = feedForwardVoltage + feedBackControllerVoltage;
        SmartDashboard.putNumber("intake/goalVelocityRadSec", goalVelocityRadSec);
        
        m_motor.setVoltage(output);
        
    }

    public double getVelocityRadSec() {
        return m_motor.getEncoder().getVelocity() * 2 * Math.PI/kPulsePerRotation * kMotorToOutputShaftRatio;
    }

    public void setVelocityRadSec(double angle) {
        goalVelocityRadSec = angle;
    }

    public boolean isSpike() {
        return m_motor.getOutputCurrent() >= kSpikeAMPS;
    }

}
