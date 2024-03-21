package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LEDController;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.RollIntakeOut;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotMidway;
import frc.robot.commands.pivot.PivotToTorus;
import frc.robot.commands.pivot.PivotTurtle;
import frc.robot.commands.Intake.RollIntakeInSlow;

public class IntakeNoteAuto extends SequentialCommandGroup {
    public IntakeNoteAuto() {
        addCommands(
            new StopIntake(),
            new PivotToTorus(),
            new RollIntakeIn(),
            new LEDController(0.61),
            new InIntake(),
            new StopIntake(),
            new PivotMidway(),
            new RollIntakeOut(), 
            new OutIntake(),
            new StopIntake(), 
            new RollIntakeInSlow(), 
            new WaitCommand(0.33),
            new StopIntake(),
            new PivotTurtle(),
            new LEDController(0.77)
        );
    }
}