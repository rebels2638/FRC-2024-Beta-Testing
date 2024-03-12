package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIONeo;

public class InShooter extends Command{
    private boolean isIn = false;

    public InShooter(){
    }
    
    @Override
    public void execute(){
        isIn = Shooter.getInstance().inShooter();
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
