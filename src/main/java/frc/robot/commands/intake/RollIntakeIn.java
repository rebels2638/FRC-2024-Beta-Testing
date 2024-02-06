package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntakeIn extends Command {
    private final Intake intakeSubsystem;
    public RollIntakeIn(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(150));
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.inIntake();
    }
}
