package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOSim extends SubsystemBase implements ElevatorIO {

    private double desiredHightMeters = 0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.hightMeters = desiredHightMeters;
    }

    @Override
    // sould be called periodically
    public void setHightMeters(double goalPositionMeters, double currentPositionMeters) {
        desiredHightMeters = goalPositionMeters;
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
    public void zeroHight() {
        desiredHightMeters = 0;
    }

}