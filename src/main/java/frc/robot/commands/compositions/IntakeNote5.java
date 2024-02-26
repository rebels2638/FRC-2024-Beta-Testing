package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.OutIntake;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.Intake.RollIntakeInSlow;
import frc.robot.commands.Intake.RollIntakeOut;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;

public class IntakeNote5 extends SequentialCommandGroup {
    public IntakeNote5(){
        addCommands(
            new ParallelRaceGroup(
                new InIntake(), 
                new RollIntakeIn()),
            new StopIntake(),
            new PivotTurtle(),
            new ParallelRaceGroup(
                new RollIntakeOut(), 
                new OutIntake()), 
            new ParallelRaceGroup(
                new RollIntakeInSlow(), 
                new WaitCommand(1)),
            new StopIntake()
        );
    }
}