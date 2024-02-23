package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindup extends Command {
    
    private double velocitySetPoint = 70;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindup(double setpoint){
        this.velocitySetPoint = setpoint;
    }

    public ShooterWindup() {
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
