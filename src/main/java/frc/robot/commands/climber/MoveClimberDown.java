package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberDown extends Command {
    private Climber climberSubsystem = Climber.getInstance();

    public MoveClimberDown() {
    }
    
    @Override
    public void initialize() { 
       climberSubsystem.setHeightMeters(0);
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
