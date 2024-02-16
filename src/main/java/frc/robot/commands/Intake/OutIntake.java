package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OutIntake extends Command{
    private Intake intakeNeo;
    private boolean isIn = false;

    public OutIntake(Intake intakeSubsystem){
        intakeNeo = intakeSubsystem;
    }
    @Override
    public void execute(){
        isIn = intakeNeo.inIntake();
    }
    @Override
    public void end(boolean isInterrupted){
        return;
    }
    @Override
    public boolean isFinished(){
        return !isIn;
    }
}
