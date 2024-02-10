package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;

public class IntakeIONeo extends SubsystemBase implements IntakeIO {
    private final double kMotorToOutputShaftRatio = 0.25;
    private CANSparkMax m_motor = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    // private Rev2mDistanceSensor distanceSensor;
    private double distanceTolerance;
    private double currentVelocityRadPerSec;
    // private DigitalInput lineBreakSensor; 

    private static final double kMAX_VOLTAGE = 12;

    public IntakeIONeo() {
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.clearFaults();
        m_motor.setInverted(true);
        // distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Rev2mDistanceSensor.Unit.kMillimeters, Rev2mDistanceSensor.RangeProfile.kDefault);
        // distanceTolerance = 0.57; //Approximate distance assuming some tolerance, CHECK AGAIN
        // distanceSensor.setEnabled(true);

        // lineBreakSensor = new DigitalInput(0);
        // distanceSensor.setAutomaticMode(true); << Probably not required but keep note that we need this if we have several of these 2m dist devices
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2; // we divide by 60 because the motor out is in RPM
        inputs.reachedSetpoint = velocityFeedBackController.atSetpoint();
        inputs.inIntake = inIntake();
        currentVelocityRadPerSec = inputs.velocityRadSec;
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec) {
        double accel = 0;
        if (goalVelocityRadPerSec > currentVelocityRadPerSec) {
            accel = 0.1;
        }
        else if (goalVelocityRadPerSec < currentVelocityRadPerSec) {
            accel = -0.1;
        }

        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, accel);
        
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
        
        m_motor.setVoltage(outVoltage);

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
    
    public boolean inIntake() {
        // TODO: WRITE THE BEAM BRAKE CODE
        //Valid range(?) check aka 2m or less


        /* //Uncomment this whenever you rhe 2mDistance sensor is there
        if(distanceSensor.isRangeValid()){
            //distanceSensor.setMeasurementPeriod();
            //Using default measurementperiod, we get its range at that moment.
            if(distanceSensor.getRange(Rev2mDistanceSensor.Unit.kMillimeters) < distanceTolerance){
                return true;
            }
            return false;
        }
        else{
            // System.out.println("Out of range");
            return false;
        }
        */

        // if(!lineBreakSensor.get()){
        //     return true; //Line is broken
        // }else{
        //     return false; //Line is NOT broken
        // }
        return false;
    }

}