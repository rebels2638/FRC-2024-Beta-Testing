package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ClimberIOSim extends SubsystemBase implements ClimberIO {

    private double climberHeightMeters;
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.climberHeightMeters = climberHeightMeters;
    }

    public void setHeightMeters(double goalPositionMeters, boolean isClimbing) {
        climberHeightMeters = goalPositionMeters;
    }

    public boolean reachedSetpoint() {
        return true;
    }
}