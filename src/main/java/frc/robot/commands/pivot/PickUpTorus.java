package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RollIntakeIn;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class PickUpTorus extends Command {
    private Intake intakeSubsystem;
    private Pivot pivotSubsystem;

    public PickUpTorus(Intake intakeSubsystem, Pivot pivotSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
    }
    @Override
    public void initialize() {
        SequentialCommandGroup group = new SequentialCommandGroup(
            new ParallelRaceGroup(new RollIntakeIn(intakeSubsystem), new PivotToTorus(pivotSubsystem)),
            new Turtle(pivotSubsystem)
        );
        group.schedule();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
