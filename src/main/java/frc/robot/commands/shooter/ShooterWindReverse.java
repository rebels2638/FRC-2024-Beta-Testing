package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindReverse extends Command {
    
    private double velocitySetPoint = -2;
    private final Shooter shooterSubsystem;

    public ShooterWindReverse(Shooter shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
