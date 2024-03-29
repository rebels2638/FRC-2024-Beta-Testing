package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindup;

public class ShootNote extends SequentialCommandGroup {
    public ShootNote() {
        addCommands(
            new ShooterWindup(),
            new WaitCommand(0.70), //Modify this later on.
            new RollIntakeIn(),
            new WaitCommand(0.4),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
