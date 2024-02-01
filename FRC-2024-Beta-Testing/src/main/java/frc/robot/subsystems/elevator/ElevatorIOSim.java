package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOSim extends SubsystemBase implements ElevatorIO {

    private double desiredHeightMeters = 0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.heightMeters = desiredHeightMeters;
    }

    @Override
    // sould be called periodically
    public void setHeightMeters(double goalPositionMeters, double currentPositionMeters) {
        desiredHeightMeters = goalPositionMeters;
    } 

    public void setVoltage(double voltage){
    }

    @Override
    public void configureController(ElevatorFeedforward pff, PIDController pfb) {
    }

    @Override
    public boolean reachedSetpoint() {
        return true;
    }

    @Override
    public void zeroHeight() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroHeight'");
    }

}