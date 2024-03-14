package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeInSlow;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.InShooter;
import frc.robot.commands.shooter.ShooterHold;
import frc.robot.commands.shooter.ShooterStop;

public class FeedAndHoldNote extends SequentialCommandGroup {
    public FeedAndHoldNote() {
        addCommands(
            new ShooterHold(),
            new WaitCommand(0.5),
            new RollIntakeInSlow(120),
            new ParallelCommandGroup(new OutIntake(), new InShooter()),
            new ParallelCommandGroup(new StopIntake(), new ShooterStop()));
    }
    void Callcancel(){
        this.cancel();
    }


}