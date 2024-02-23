package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindReverse extends Command {
    private double velocitySetPoint = -9;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindReverse(){
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
