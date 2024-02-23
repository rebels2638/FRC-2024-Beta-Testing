package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterHold extends Command {
    // private double velocitySetPoint = 3;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterHold(){
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(5);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
