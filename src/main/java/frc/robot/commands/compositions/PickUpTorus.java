package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.pivot.PivotToTorus;

public class PickUpTorus extends SequentialCommandGroup {
    public PickUpTorus() {
        //Race the rolling intake in and InIntake, rolling intake never finishes, therefore it only ends when the InIntake ends.
        addCommands(
            new ParallelCommandGroup(new ParallelRaceGroup(new RollIntakeIn(), new InIntake()),
            new PivotToTorus())
        );
    }
    }
