package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;

public class ShootNoteAuto extends SequentialCommandGroup {
    public ShootNoteAuto() {
        addCommands(
            // new WaitCommand(0.2),
            new RollIntakeIn(),
            new WaitCommand(0.3),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
