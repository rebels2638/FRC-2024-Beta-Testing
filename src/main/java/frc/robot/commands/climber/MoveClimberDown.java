package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberDown extends Command {
    private Climber climberSubsystem;

    public MoveClimberDown(Climber climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    
    @Override
    public void execute() { 
       climberSubsystem.setHeightMeters(0.0); 
    }
    @Override
    public void end(boolean isInterrupted){
        //Add end if need be
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.reachedSetpoint();
    }
}
