package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIONeo;

public class ShooterWindup extends Command {
    //Variables
    private double velocitySetPoint;
    private ShooterIONeo Shooter;
    ShooterWindup(ShooterIONeo subsystem, double setPoint){
        velocitySetPoint = setPoint;
        Shooter = subsystem;
        addRequirements(Shooter);
    }
    @Override
    public void execute(){
        Shooter.setVelocityRadSec(velocitySetPoint, 0);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
