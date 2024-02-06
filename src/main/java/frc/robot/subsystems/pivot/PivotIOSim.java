package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotIOSim extends SubsystemBase implements PivotIO {

    double desiredPositionRad = 0; 
    double desiredPositionDeg = 0;
    double desiredVelocityRadSec = 0;
    double desiredVelocityDegSec = 0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRad = desiredPositionRad;
        inputs.positionDeg = desiredPositionDeg;

        inputs.velocityRadSec = desiredVelocityRadSec;
        inputs.velocityDegSec = desiredVelocityDegSec;

        inputs.reachedSetpoint = true;
    }

    @Override
    // sould be called periodically
    public void setPosition(double goalPositionRad, double currentRadAngle) {
        desiredPositionRad = goalPositionRad;
        desiredPositionDeg = Math.toDegrees(goalPositionRad);
    } 

    @Override
    // sould be called periodically
    public void setVelocity(double goalVelocityRadPerSec, double currentVelocityRadPerSec) {
        desiredVelocityRadSec = goalVelocityRadPerSec;
        desiredVelocityDegSec = Math.toDegrees(goalVelocityRadPerSec);
    } 

    public void setVoltage(double voltage){
    }

    @Override
    public void configureController(ArmFeedforward pff, PIDController pfb, 
                                        ArmFeedforward vff, PIDController vfb ) {
    }

    @Override
    public void zeroAngle() {
    }

}