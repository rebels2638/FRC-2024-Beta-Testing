package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class InShooter extends Command{
    private Intake intakeSubsystem;
    private boolean isIn = false;

    public InShooter(Intake intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }
    @Override
    public void execute(){
        isIn = !intakeSubsystem.inIntake();
    }
    @Override
    public void end(boolean isInterrupted){
        return;
    }
    @Override
    public boolean isFinished(){
        return isIn;
    }
}
