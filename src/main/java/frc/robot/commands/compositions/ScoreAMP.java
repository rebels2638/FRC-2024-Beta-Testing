package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.RollIntakeInSlow;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.InShooter;
import frc.robot.commands.shooter.ShooterHold;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindReverse;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ScoreAMP extends SequentialCommandGroup {

    public ScoreAMP() {
        addCommands(
            new ShooterStop(), // for what reason??
            new MoveElevatorAMP(),
            new WaitCommand(0.3),
            new ShooterWindReverse(),
            new WaitCommand(1),
            new ShooterStop(),
            new MoveElevatorTurtle()
        );
    }
}
