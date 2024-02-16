
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class RollIntakeIn extends Command {
    private final Intake intakeSubsystem;
    private final Pivot pivotSubsystem;
    public RollIntakeIn(Intake intakeSubsystem, Pivot pivotSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        if (pivotSubsystem.getDegAngle() < 45) {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 17)); //Use radians directly.
        }
        else {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 10)); //Use radians directly.
        }
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
