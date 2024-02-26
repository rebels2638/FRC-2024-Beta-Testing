package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindReverse;

public class ScoreAMP extends SequentialCommandGroup {

    public ScoreAMP() {
        addCommands(
            new ShooterStop(), // for what reason??
            new MoveElevatorAMP(),
            new WaitCommand(0.3),
            new ShooterWindReverse(),
            new WaitCommand(0.8),
            new ShooterStop(),
            new MoveElevatorTurtle()
        );
    }
}
