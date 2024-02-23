package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotMidway extends Command {
    private final Pivot pivotSubsystem = Pivot.getInstance();

    public PivotMidway() {
    }

    @Override
    public void initialize() { 
        pivotSubsystem.setDegAngle(33);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.reachedSetpoint();
    }
}
