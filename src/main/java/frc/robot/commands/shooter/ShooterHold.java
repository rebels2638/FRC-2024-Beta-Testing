package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterHold extends Command {
    
    private double velocitySetPoint = 0.5;
    private final Shooter shooterSubsystem;

    public ShooterHold(Shooter shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.setVelocityRadSec(0.5);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
