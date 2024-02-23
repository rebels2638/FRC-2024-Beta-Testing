package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class StopIntake extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();
    
    public StopIntake() {
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(0)); //Use radians directly.
    }
    
    @Override
    public void end(boolean isInterrupted){
        intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.reachedSetpoint();
    }
}
