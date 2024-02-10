
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class StopIntake extends Command {
    private final Intake intakeSubsystem;
    public StopIntake(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(0)); //Use radians directly.
        // System.out.println("CALLLEELDLEDLAKDJASBKND");
    }
    @Override
    public void end(boolean isInterrupted){
        intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
