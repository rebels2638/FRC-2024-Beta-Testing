package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterWindupLob extends Command {
    
    private double velocitySetPoint = 48;
    private final Shooter shooterSubsystem = Shooter.getInstance();

    public ShooterWindupLob(double setpoint){
        this.velocitySetPoint = setpoint;
    }

    public ShooterWindupLob() {
    }
    public ShooterWindupLob(SequentialCommandGroup s){
        s.cancel();
    }

    @Override
    public void initialize(){
        shooterSubsystem.setVelocityRadSec(velocitySetPoint, false, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
