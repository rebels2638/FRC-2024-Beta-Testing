package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command ;
import frc.robot.subsystems.pivot.Pivot;

public class PivotToCube extends Command {

    private final Pivot pivotSubsystem;
    public PivotToCube(Pivot pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() { 
        pivotSubsystem.setDegAngle(90);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
