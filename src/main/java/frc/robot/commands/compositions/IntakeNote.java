package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.RollIntakeOut;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.Intake.RollIntakeInSlow;

public class IntakeNote extends SequentialCommandGroup {
    public IntakeNote() {
        addCommands(
            new PivotToTorus(),  
            new RollIntakeIn(),
            new InIntake(),
            new StopIntake(),
            new PivotTurtle(),
            new RollIntakeOut(), 
            new OutIntake(),
            new StopIntake(), 
            new RollIntakeInSlow(), 
            new WaitCommand(0.8),
            new StopIntake()
        );
    }
}