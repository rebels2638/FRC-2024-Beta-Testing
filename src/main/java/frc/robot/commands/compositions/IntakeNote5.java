package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
//Please make all composition commands 
//Use this as an example for Singleton class structure (Standard) sequentialCommandGroup creation; {The command runs anyways through the same code in the robotcontainer}, this is just much simpler.
import frc.robot.subsystems.shooter.Shooter;

public class IntakeNote5 extends SequentialCommandGroup {
    public IntakeNote5(){
        addCommands(
           new ParallelDeadlineGroup(new InIntake(Intake.getInstance()), new RollIntakeIn(Intake.getInstance(), Pivot.getInstance())),
            new StopIntake(Intake.getInstance()),
            new PivotTurtle(Pivot.getInstance()),
            new ParallelRaceGroup(new RollIntakeOut(Intake.getInstance()), new OutIntake(Intake.getInstance())), 
            new ParallelRaceGroup(new RollIntakeInSlow(Intake.getInstance()), new WaitCommand(1)),
            new StopIntake(Intake.getInstance()));
    }
}