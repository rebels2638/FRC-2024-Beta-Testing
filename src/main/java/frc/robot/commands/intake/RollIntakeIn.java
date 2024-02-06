<<<<<<<< HEAD:src/main/java/frc/robot/commands/Intake/RollIntake.java
package frc.robot.commands.Intake;
========
package frc.robot.commands.intake;
>>>>>>>> 5cdc2de8944ad8ef3f84ad3968e0e3f4e35c16a2:src/main/java/frc/robot/commands/intake/RollIntakeIn.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntakeIn extends Command {
    private final Intake intakeSubsystem;
    public RollIntakeIn(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() { 
        intakeSubsystem.setVelocityRadSec(2*Math.PI); //Use radians directly.
    }
    @Override
    public void end(boolean isInterrupted){
        intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.inIntake();
    }
}
