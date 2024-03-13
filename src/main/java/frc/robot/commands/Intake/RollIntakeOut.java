
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntakeOut extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();

    public RollIntakeOut() {
    }

    @Override
    public void initialize() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(-150)); //Use radians directly.
    }
    
    @Override
    public void end(boolean isInterrupted){
       //ta inkeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return true;

    }
}
