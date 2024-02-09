package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.RollIntakeIn;
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
            //Race the rolling intake in and InIntake, rolling intake never finishes, therefore it only ends when the InIntake ends. 
            new ParallelCommandGroup(new ParallelRaceGroup(new RollIntakeIn(intakeSubsystem, pivotSubsystem), new InIntake(intakeSubsystem)),
             new PivotToTorus(pivotSubsystem))
        );
        group.schedule();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
