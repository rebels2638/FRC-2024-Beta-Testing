package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.commands.elevator.MoveElevatorAMP;
//Please make all composition commands 
//Use this as an example for Singleton class structure (Standard) sequentialCommandGroup creation; {The command runs anyways through the same code in the robotcontainer}, this is just much simpler.
import frc.robot.commands.pivot.PivotToTorus;

public class Climb extends SequentialCommandGroup {
    public Climb() {
        addCommands(
            new SequentialCommandGroup(
                new MoveElevatorAMP(),
                new PivotToTorus(),
                new WaitCommand(3),
                new MoveClimberUp(),
                new WaitCommand(3),
                new MoveClimberDown()
            )
        );

    }
}