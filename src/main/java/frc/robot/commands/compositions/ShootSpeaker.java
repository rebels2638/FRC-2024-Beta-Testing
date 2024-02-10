package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShootSpeaker extends Command {
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;
    private final Pivot pivotSubsystem;

    public ShootSpeaker(Shooter shooterSubsystem, Intake intakeSubsystem, Pivot pivotSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem, pivotSubsystem);
    }

    @Override
    public void initialize() {
        // SequentialCommandGroup(new ParallelDeadlineGroup(new RollIntakeIn(intakeSubsystem, pivotSubsystem), new InstantCommand() ));
    }
}
