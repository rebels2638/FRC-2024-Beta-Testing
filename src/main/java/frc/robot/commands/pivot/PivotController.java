package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.lib.input.XboxController;

public class PivotController extends Command {

    Pivot pivotSubsystem = null;
    XboxController controller;

    public PivotController(Pivot pivotSubsystem, XboxController controller) {
        this.pivotSubsystem = pivotSubsystem;
        this.controller = controller;
        addRequirements(pivotSubsystem);
    }
    @Override
    public void execute() { 
    //     pivotSubsystem.setVelocityControlMode(true);
    //    pivotSubsystem.setVelocitySetPoint(0.1 * controller.getRightY());
       pivotSubsystem.setVoltage(12 * controller.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    public void end(boolean interrupted){
        System.out.println("CALLED END");
        pivotSubsystem.setVelocitySetPoint(0);
        return;
    }
}

