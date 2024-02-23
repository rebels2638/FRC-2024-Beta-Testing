package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterStop extends Command {
    // private double velocitySetPoint = 0;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterStop(){
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
