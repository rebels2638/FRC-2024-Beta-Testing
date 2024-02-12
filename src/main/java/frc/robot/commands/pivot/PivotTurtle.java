package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotTurtle extends Command {

    private final Pivot pivotSubsystem;
    public PivotTurtle(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() { 
        pivotSubsystem.setDegAngle(0);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}