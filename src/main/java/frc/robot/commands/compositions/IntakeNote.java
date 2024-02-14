package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class IntakeNote extends Command {
    private final Intake intakeSubsystem;
    private final Pivot pivotSubsystem;

    public IntakeNote(Intake intakeSubsystem, Pivot pivotSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(intakeSubsystem, pivotSubsystem);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.inIntake() == false) {
            SequentialCommandGroup commandGroup = new SequentialCommandGroup(new PivotToTorus(pivotSubsystem), 
            new ParallelDeadlineGroup(new InIntake(intakeSubsystem), new RollIntakeIn(intakeSubsystem, pivotSubsystem)),
            new StopIntake(intakeSubsystem),
            new PivotTurtle(pivotSubsystem));

            commandGroup.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
