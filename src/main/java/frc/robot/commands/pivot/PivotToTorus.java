package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class PivotToTorus extends Command {

    private final Pivot pivotSubsystem;
    public PivotToTorus(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() { 
        pivotSubsystem.setDegAngle(78);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
