package frc.robot.commands.shooter;

import frc.robot.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterToggle extends Command {
    
    private final double stopVelocitySetPoint = 0;
    private final double windupVelocitySetpoint = 5;
    private final double windupReverseVelocitySetpoint = -2;
    private final double holdVelocitySetpoint = 0;
    private int tapped;

    private final Shooter shooterSubsystem = Shooter.getInstance();
    private XboxController m_controller;

    public ShooterToggle(XboxController controller){
        this.m_controller = controller;
        this.tapped = 0;
    }

    @Override
    public void execute() {
        if (this.m_controller.getLeftBumper().getAsBoolean()) {
            this.tapped++;
            double desiredSpeed = 0;
            
            switch(tapped) {
                case 1:
                    desiredSpeed = this.windupVelocitySetpoint;
                    break;

                case 2:
                    desiredSpeed = this.stopVelocitySetPoint;
                    break;

                case 3:
                    desiredSpeed = this.windupReverseVelocitySetpoint;
                    break;

                case 4:
                    desiredSpeed = this.holdVelocitySetpoint;
                    this.tapped = 0;
                    break;
            }
            
            shooterSubsystem.setVelocityRadSec(desiredSpeed);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
    
}
