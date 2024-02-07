package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindup extends Command {
    
    private double velocitySetPoint;
    private final Shooter shooterSubsystem;

    ShooterWindup(Shooter shooterSubsystem, double setPoint){
        velocitySetPoint = setPoint;
        this.shooterSubsystem = shootersubsystem;
        addRequirements(shooterSubsystem);
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
