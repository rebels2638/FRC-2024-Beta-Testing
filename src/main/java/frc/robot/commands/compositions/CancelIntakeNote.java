package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RollIntakeEject;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.pivot.PivotTurtle;

public class CancelIntakeNote extends SequentialCommandGroup {
    public CancelIntakeNote(SequentialCommandGroup c, SequentialCommandGroup c2) {
        if(c != null && c2 != null){
        addCommands(
            new InstantCommand(()->c.cancel()),
            new InstantCommand(()->c2.cancel()),
                new StopIntake(),
                new PivotTurtle());
        }
        else if(c != null){
            addCommands(
                new InstantCommand(()->c.cancel()),
                new StopIntake(), 
                new PivotTurtle()
                );
        }
        else if(c2 != null)
        {
            addCommands(
                new InstantCommand(()->c2.cancel()),
                new StopIntake(), 
                new PivotTurtle()
                );
        }
        else{
            addCommands(
                new StopIntake(), 
                new PivotTurtle()
                );
        }
    }
}