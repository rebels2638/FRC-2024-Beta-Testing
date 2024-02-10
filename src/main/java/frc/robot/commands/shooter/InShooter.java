package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class InShooter extends Command{
    private Shooter shooterSubsystem;
    private boolean isIn = false;

    public InShooter(Shooter shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void execute(){
        isIn = shooterSubsystem.inShooter();
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
