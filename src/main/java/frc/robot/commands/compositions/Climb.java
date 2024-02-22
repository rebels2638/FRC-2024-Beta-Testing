package frc.robot.commands.compositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.commands.elevator.MoveElevatorAMP;
//Please make all composition commands 
//Use this as an example for Singleton class structure (Standard) sequentialCommandGroup creation; {The command runs anyways through the same code in the robotcontainer}, this is just much simpler.

public class Climb extends SequentialCommandGroup {
    public Climb() {
        addCommands(
            new SequentialCommandGroup(
                new MoveElevatorAMP(Elevator.getInstance()),
                new WaitCommand(3),
                new MoveClimberUp(Climber.getInstance()),
                new WaitCommand(3),
                new MoveClimberDown(Climber.getInstance())
            )
        );

    }
}