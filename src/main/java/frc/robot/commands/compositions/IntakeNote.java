package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.RollIntakeOut;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
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
            new WaitCommand(1),
            new StopIntake()
        );
    }
}