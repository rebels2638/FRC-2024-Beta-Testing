package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class InIntake extends Command{
    private boolean isIn = false;

    public InIntake(){
    }
    
    @Override
    public void execute(){
        isIn = Intake.getInstance().inIntake();
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
