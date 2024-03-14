package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindReverse;

public class ScoreAMP extends SequentialCommandGroup {

    public ScoreAMP() {
        addCommands(
            // new ShooterStop(), // for what reason??
            // new MoveElevatorAMP(),
            // new WaitCommand(0.6),
            // new ShooterWindReverse(),
            // new WaitCommand(0.8),
            // new ShooterStop(),
            // new MoveElevatorTurtle()
            new ShooterWindReverse(),
            new WaitCommand(0.8),
            new ShooterStop()
        );
    }

    // public ScoreAMP() {
    //     addCommands(
    //         new FeedAndHoldNote(),
    //         new ShooterStop(),
    //         new MoveElevatorAMP(),
    //         new WaitCommand(0.6),
    //         new ShooterWindReverse(),
    //         new WaitCommand(0.4),
    //         new ShooterStop(),
    //         new MoveElevatorTurtle()
    //     );
    // }

}
