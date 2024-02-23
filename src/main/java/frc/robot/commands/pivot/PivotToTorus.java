package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotToTorus extends Command {
    private final Pivot pivotSubsystem = Pivot.getInstance();

    public PivotToTorus() {
    }

    @Override
    public void initialize() { 
        pivotSubsystem.setDegAngle(87);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}
