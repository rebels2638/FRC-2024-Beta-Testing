package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberUp extends Command {
    private Climber climberSubsystem = Climber.getInstance();

    public MoveClimberUp() {
    }
    
    @Override
    public void execute() { 
       climberSubsystem.setHeightMeters(0.51);
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
