package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShootNote extends SequentialCommandGroup {
    public ShootNote() {
        addCommands(
            new ShooterWindup(),
            new WaitCommand(0.7), //Modify this later on.
            new RollIntakeIn(),
            new WaitCommand(0.5),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
