package frc.robot.commands.pivot;

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
        intakeSubsystem.setVelocityRadSec(Math.toRadians(150));
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.inIntake();
    }
}
