package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;

public class IntakeIONeo extends SubsystemBase implements IntakeIO {
    private static final double kMotorToOutputShaftRatio = 0.01;
    private CANSparkMax m_motor = new CANSparkMax(21, MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    private Rev2mDistanceSensor distanceSensor;
    private double distanceTolerance;

    private static final double kMAX_VOLTAGE = 12;

    public IntakeIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults();
        m_motor.setInverted(true);
        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Rev2mDistanceSensor.Unit.kMillimeters, Rev2mDistanceSensor.RangeProfile.kDefault);
        distanceTolerance = 0.5; //0.5 meters TODO: change this value to actually fit the distance
        distanceSensor.setEnabled(true);
        // distanceSensor.setAutomaticMode(true); << Probably not required but keep note that we need this if we have several of these 2m dist devices
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, 0);
        
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);

        double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
            
        if (outVoltage > kMAX_VOLTAGE) {
            outVoltage = 12;
        }
        else if (outVoltage < -kMAX_VOLTAGE) {
            outVoltage = -12;
        }
        Logger.recordOutput("Intake/voltageOut", outVoltage);
        
        m_motor.setVoltage(feedForwardVoltage + feedBackControllerVoltage);

    } 

    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    @Override
    public boolean reachedSetpoint() {
        return velocityFeedBackController.atSetpoint();
    }

    @Override
    public boolean inIntake() {
        // TODO: WRITE THE BEAM BRAKE CODE
        if(distanceSensor.isRangeValid()){
            //distanceSensor.setMeasurementPeriod();
            if(distanceSensor.getRange(Rev2mDistanceSensor.Unit.kMillimeters) < distanceTolerance){
                return true;
            }
            return false;
        }
        else{
            System.out.println("Out of range");
            return false;
        }
        
    }

}