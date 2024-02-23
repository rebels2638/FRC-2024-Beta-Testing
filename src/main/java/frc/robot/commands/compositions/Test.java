package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RollIntakeIn;
import frc.robot.commands.shooter.ShooterWindup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
//Please make all composition commands 
//Use this as an example for Singleton class structure (Standard) sequentialCommandGroup creation; {The command runs anyways through the same code in the robotcontainer}, this is just much simpler.
import frc.robot.subsystems.shooter.Shooter;

public class Test extends SequentialCommandGroup {
    public Test(){
        addCommands(
            new SequentialCommandGroup(new RollIntakeIn(),
         new ShooterWindup()));

    }
}