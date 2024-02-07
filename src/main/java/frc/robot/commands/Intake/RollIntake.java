
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntake extends Command {
    private final Intake intakeSubsystem;
    public RollIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(2*Math.PI); //Use radians directly.
    }
    @Override
    public void end(boolean isInterrupted){
        intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.inIntake();
    }
}
