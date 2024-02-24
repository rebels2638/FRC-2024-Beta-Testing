package frc.robot.commands.Intake;

import frc.robot.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class IntakeToggle extends Command {
    
    private int tapped;
    private final Intake intakeSubsystem = Intake.getInstance();
    private final Pivot pivotSubsystem = Pivot.getInstance();
    private XboxController m_controller;

    public IntakeToggle(XboxController controller){
        this.m_controller = controller;
        this.tapped = 0;
    }

    @Override
    public void execute() {
        if (this.m_controller.getYButton().getAsBoolean()) {
            this.tapped++;
            
            switch(tapped) {
                case 1: // roll
                    if (pivotSubsystem.getDegAngle() < 45) {
                        intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 20)); //Use radians directly.
                    } else {
                        intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 4)); //Use radians directly.
                    }
                    break;
                    
                case 2: // stop
                    intakeSubsystem.setVelocityRadSec(0);
                    break;

                case 3: // reverse
                    intakeSubsystem.setVelocityRadSec(-1);
                    this.tapped = 0;
                    break;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return intakeSubsystem.reachedSetpoint();
    }
}
