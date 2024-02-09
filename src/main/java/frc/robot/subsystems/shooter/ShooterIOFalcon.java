package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;

// import com.revrobotics.CANSparkMaxLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIOFalcon extends SubsystemBase implements ShooterIO {
    private static final double kMotorToOutputShaftRatio = 1; //Last Checked 2/6/2024
    private TalonFX m_motor1 = new TalonFX(13); //TODO: Get Motor IDs
    private TalonFX m_motor2 = new TalonFX(14); 

    private PIDController velocityFeedBackController = new PIDController(6, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    private Rev2mDistanceSensor distanceSensor;
    private double distanceTolerance;
    private double currentVelocityRadPerSec;

    private static final double kMAX_VOLTAGE = 12;

    public ShooterIOFalcon() {
        m_motor1.setNeutralMode(NeutralModeValue.Brake);
        m_motor1.clearStickyFault_BootDuringEnable();
        m_motor1.setInverted(false);

        m_motor2.setNeutralMode(NeutralModeValue.Brake);
        m_motor2.clearStickyFault_BootDuringEnable(); //Don't Do this unless in competition
        m_motor2.setInverted(false);
        // distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Rev2mDistanceSensor.Unit.kMillimeters, Rev2mDistanceSensor.RangeProfile.kDefault);
       //  distanceTolerance = 0.4572; //0.5 meters TODO: change this value to actually fit the distance I only put an Approximation
        // distanceSensor.setEnabled(true);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = m_motor1.getVelocity().getValueAsDouble()*2*Math.PI*kMotorToOutputShaftRatio; // we divide by 60 because the motor out is in RPM
        currentVelocityRadPerSec = inputs.velocityRadSec;
        inputs.reachedSetpoint = velocityFeedBackController.atSetpoint();
        inputs.inShooter = isInShooter();
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec) {
        // double ffVelo = 0;
        // if (goalVelocityRadPerSec > currentVelocityRadPerSec) {
        //     ffVelo = 1;
        // }
        // else if (goalVelocityRadPerSec < currentVelocityRadPerSec) {
        //     ffVelo = -1;
        // }

        // double feedForwardVoltage = velocityFeedForwardController.calculate(currentVelocityRadPerSec 0 );
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(goalVelocityRadPerSec - currentVelocityRadPerSec);
        // double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
        double outVoltage = feedBackControllerVoltage;
            
        if (outVoltage > kMAX_VOLTAGE) {
            outVoltage = 12;
        }
        else if (outVoltage < -kMAX_VOLTAGE) {
            outVoltage = -12;
        }
        if (goalVelocityRadPerSec > 0) {
             outVoltage = 6;
        }
        else {
            outVoltage = 0;
        }
        Logger.recordOutput("Shooter/voltageOut", outVoltage);
        m_motor1.setVoltage(outVoltage);
        m_motor2.setVoltage(outVoltage);

    } 

    @Override
    public void setVoltage(double voltage){
        m_motor1.setVoltage(voltage);
    }

    @Override
    public void configureController(SimpleMotorFeedforward vff, PIDController vfb ) {
        velocityFeedBackController = vfb;
        velocityFeedForwardController = vff;
    }

    private boolean isInShooter(){
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
    }

}