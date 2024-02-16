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
       elevatorSubsystem.setHeightMeters(0.4, true, false);
    }
    @Override
    public void end(boolean isInterrupted){
        //Add end if need be
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.reachedSetpoint();
    }
}
