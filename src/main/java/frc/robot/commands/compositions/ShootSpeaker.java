package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.commands.shooter.AlignWithTargetPoint;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.elevator.MoveElevatorTurtle;
import frc.robot.commands.pivot.PivotTurtle;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.limelight.PoseLimelight;

import frc.robot.Utils.RebelUtil;

public class ShootSpeaker extends Command {
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;
    private final Pivot pivotSubsystem;
    private final Elevator elevatorSubsystem;
    private final PoseLimelight visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    public ShootSpeaker(Shooter shooterSubsystem, Intake intakeSubsystem, Pivot pivotSubsystem, Elevator elevatorSubsystem, PoseLimelight visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem, pivotSubsystem, elevatorSubsystem, visionSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {
        SequentialCommandGroup commandGroup = 
        new SequentialCommandGroup(
            new ParallelRaceGroup(new AlignWithTargetPoint(this.shooterSubsystem, this.swerveSubsystem, this.visionSubsystem), new ShooterWindup(this.shooterSubsystem, RebelUtil.flywheelSpeed)),
            new ParallelCommandGroup(new MoveElevatorTurtle(this.elevatorSubsystem), new PivotTurtle(this.pivotSubsystem)),
            new ParallelRaceGroup(new RollIntakeIn(this.intakeSubsystem, this.pivotSubsystem), new InIntake(this.intakeSubsystem)),
            new StopIntake(this.intakeSubsystem),
            new ShooterStop(this.shooterSubsystem)
        );
        commandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
