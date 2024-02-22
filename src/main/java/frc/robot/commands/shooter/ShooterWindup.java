package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindup extends Command {
    
    private double velocitySetPoint = 70;
    private final Shooter shooterSubsystem;

    public ShooterWindup(Shooter shooterSubsystem, double setpoint){
        this.shooterSubsystem = shooterSubsystem;
        this.velocitySetPoint = setpoint;
        addRequirements(shooterSubsystem);
    }

    public ShooterWindup(Shooter shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return shooterSubsystem.reachedSetpoint();
    }
    
}
