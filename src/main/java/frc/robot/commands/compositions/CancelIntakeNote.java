package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.ShooterStop;

public class CancelIntakeNote extends SequentialCommandGroup {
    public CancelIntakeNote() {
        addCommands(
            new ParallelRaceGroup(
            new ParallelCommandGroup(
                new StopIntake(),
                new PivotTurtle(),
                new ShooterStop()
            ),
            new WaitCommand(1))
        );
    }
}