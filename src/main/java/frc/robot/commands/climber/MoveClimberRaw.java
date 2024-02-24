package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberRaw extends Command {
    private Climber climberSubsystem = Climber.getInstance();
    private XboxController controller;

    public MoveClimberRaw(Climber c, XboxController x) {
        controller = x;
        climberSubsystem = c;
        addRequirements(c);
    }
    
    @Override
    public void execute() { 
       climberSubsystem.setVoltage(-controller.getLeftY() * 12); 
    }
    
    @Override
    public void end(boolean isInterrupted){
              climberSubsystem.setVoltage(0); 

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
