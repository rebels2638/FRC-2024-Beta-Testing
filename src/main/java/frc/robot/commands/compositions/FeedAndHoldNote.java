package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeInSlow;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.ShooterHold;
import frc.robot.commands.shooter.ShooterStop;

public class FeedAndHoldNote extends SequentialCommandGroup {
    public FeedAndHoldNote() {
        addCommands(
            new RollIntakeInSlow(),
            new ShooterHold(),
            new OutIntake(),
            new ParallelCommandGroup(new StopIntake(), new ShooterStop()));
    }
}