package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorTurtle extends Command {
    private Elevator elevatorSubsystem;

    public MoveElevatorTurtle(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }
    
    @Override
    public void execute() { 
       elevatorSubsystem.setHeightMeters(0.01, true, false);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.reachedSetpoint();
    }
}
