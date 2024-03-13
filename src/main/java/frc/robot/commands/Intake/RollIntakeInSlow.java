
package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RollIntakeInSlow extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();
    private double velo;
    public RollIntakeInSlow() {
        velo = 150;
    }
    public RollIntakeInSlow(double a){
        velo = a;

    }

    @Override
    public void initialize() { 
        intakeSubsystem.setVelocityRadSec(Math.toRadians(velo)); //Use radians directly.
    }
    
    @Override
    public void end(boolean isInterrupted){
        // intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
