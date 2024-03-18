package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindup extends Command {
    
    private double velocitySetPoint = 60;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindup(double setpoint){
        this.velocitySetPoint = setpoint;
    }

    public ShooterWindup() {
    }
    public ShooterWindup(SequentialCommandGroup s){
        s.cancel();
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
