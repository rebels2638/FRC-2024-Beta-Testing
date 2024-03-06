package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;

// import com.revrobotics.CANSparkMaxLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.RebelUtil;

public class ShooterIONeo extends SubsystemBase implements ShooterIO {
    private static final double kMotorToOutputShaftRatio = 1; //Last Checked 2/6/2024
    private double wheelVelo = 0.0;
    private CANSparkMax m_motor1 = new CANSparkMax(13, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax m_motor2 = new CANSparkMax(14, CANSparkMax.MotorType.kBrushless); 

    private PIDController velocityFeedBackController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward velocityFeedForwardController = new SimpleMotorFeedforward(0, 0, 0);

    private double distanceTolerance;
    private double currentVelocityRadPerSec;

    private static final double kMAX_VOLTAGE = 12;

    private final Rev2mDistanceSensor distanceSensor;

    public ShooterIONeo() {
        m_motor1.clearFaults();
        m_motor2.clearFaults();

        m_motor1.setInverted(false);
        m_motor2.setInverted(false); // TODO: check inversions.. def wrong
        // m_motor2.follow(m_motor1);

        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Rev2mDistanceSensor.Unit.kMillimeters, Rev2mDistanceSensor.RangeProfile.kDefault);
         distanceTolerance = 0.4572; 
        distanceSensor.setEnabled(true);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocityRadSec = m_motor1.getEncoder().getVelocity() / 60 * kMotorToOutputShaftRatio * Math.PI * 2; // we divide by 60 because the motor out is in RPM
        currentVelocityRadPerSec = inputs.velocityRadSec;
        inputs.reachedSetpoint = velocityFeedBackController.atSetpoint();
        // inputs.inShooter = isInShooter(); //What if the point of this.
    }

    @Override
    // should be called periodically
    public void setVelocityRadSec(double goalVelocityRadPerSec) {
        double ffVelo = 0;
        if (goalVelocityRadPerSec > currentVelocityRadPerSec) {
            ffVelo = 1; 
        }
        else if (goalVelocityRadPerSec < currentVelocityRadPerSec) {
            ffVelo = -1; 
        }

        double feedForwardVoltage = velocityFeedForwardController.calculate(goalVelocityRadPerSec, ffVelo);
        velocityFeedBackController.setSetpoint(goalVelocityRadPerSec);
        double feedBackControllerVoltage = velocityFeedBackController.calculate(currentVelocityRadPerSec);
        double outVoltage = feedForwardVoltage + feedBackControllerVoltage;
        // double outVoltage = feedBackControllerVoltage;
            
        outVoltage = RebelUtil.constrain(outVoltage, -12, 12);
        // if (goalVelocityRadPerSec > 0) {
        //      outVoltage = 6;
        // }
        // else {
        //     outVoltage = 0;
        // }
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
        // System.out.println("Out of range");
        return false;
    }

}