package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.lib.RebelUtil;
import frc.robot.lib.input.XboxController;


public class ElevatorControlRaw extends Command{
    Elevator ElevatorSubsystem;
    XboxController controller;
    /**
     * This is really for testing, and a raw controller for the operator to use whenever they MUST, not recommend for use when other commands are working
     * @param Elevator
     * @param m_controller
     */
    public ElevatorControlRaw(Elevator Elevator, XboxController m_controller){
        ElevatorSubsystem = Elevator; 
        controller = m_controller;
        addRequirements(Elevator);
    }
    @Override
    public void execute(){
        ElevatorSubsystem.setVoltage(RebelUtil.linearDeadband(controller.getRightY(), 0.08) * 12);
    }
    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
