package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorAMP extends Command {
    private Elevator elevatorSubsystem;

    public MoveElevatorAMP(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }
    
    @Override
    public void execute() { 
       elevatorSubsystem.setHightMeters(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
