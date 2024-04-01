package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.subsystems.shooter.Shooter;;

public class ShootNoteVariable extends SequentialCommandGroup {
    public ShootNoteVariable() {
        addCommands(
            // new WaitCommand(0.2),
            // new PivotTurtle(),,
            new WaitCommand(0.25),
            new RollIntakeIn(),
            new WaitCommand(0.3),
            new StopIntake()
        );
    }
}
