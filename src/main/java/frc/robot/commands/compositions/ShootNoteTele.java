package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.shooter.ShooterStop;

public class ShootNoteTele extends SequentialCommandGroup {
    public ShootNoteTele() {
        addCommands(
             //Modify this later on.
            new RollIntakeIn(),
            new WaitCommand(0.5),
            new StopIntake(),
            new ShooterStop()
        );
    }


}
