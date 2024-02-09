package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterStop extends Command {
    
    private double velocitySetPoint = 0;
    private final Shooter shooterSubsystem;

    public ShooterStop(Shooter shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
