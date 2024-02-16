package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.elevator.MoveElevatorAMP;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.shooter.InShooter;
import frc.robot.commands.shooter.ShooterHold;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.ShooterWindReverse;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ScoreAMP extends Command {
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;
    private final Pivot pivotSubsystem;
    private final Elevator elevatorSubsystem;

    public ScoreAMP(Shooter shooterSubsystem, Intake intakeSubsystem, Pivot pivotSubsystem, Elevator elevatorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        
        addRequirements(shooterSubsystem, intakeSubsystem, pivotSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new PivotTurtle(pivotSubsystem),
            new ParallelRaceGroup(
                new RollIntakeIn(intakeSubsystem, pivotSubsystem), 
                new WaitCommand(0.1)),
            new ParallelRaceGroup(
                new OutIntake(intakeSubsystem),
                new RollIntakeIn(intakeSubsystem, pivotSubsystem), 
                new ShooterHold(shooterSubsystem)),
            new StopIntake(intakeSubsystem),
            new ShooterStop(shooterSubsystem),
            new MoveElevatorAMP(elevatorSubsystem),
            new ShooterWindReverse(shooterSubsystem)
            );

        commandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
