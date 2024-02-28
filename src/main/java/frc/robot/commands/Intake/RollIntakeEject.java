
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntakeEject extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();

    public RollIntakeEject() {
    }

    @Override
    public void initialize() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(-180)); //Use radians directly.
    }
    
    @Override
    public void end(boolean isInterrupted){
        // intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
