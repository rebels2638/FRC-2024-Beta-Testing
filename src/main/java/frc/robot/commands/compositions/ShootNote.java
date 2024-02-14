package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShootNote extends Command {
    private final Intake intakeSubsystem;
    private final Shooter shooterSubsystem;
    private final Pivot pivotSubsystem;

    public ShootNote(Intake intakeSubsystem, Shooter shooterSubsystem, Pivot pivotSubsystem) {
        this.intakeSubsystem = intakeSubsystem; 
        this.shooterSubsystem = shooterSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.inIntake() == true) {
            SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new WaitCommand(0.7),
                new ParallelDeadlineGroup(
                    new InIntake(intakeSubsystem),
                    new RollIntakeIn(intakeSubsystem, pivotSubsystem), 
                    new ShooterWindup(shooterSubsystem)));

            commandGroup.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
