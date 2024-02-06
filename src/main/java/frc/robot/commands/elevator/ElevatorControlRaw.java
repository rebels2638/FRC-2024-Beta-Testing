package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorIONeo;
import frc.robot.lib.input.XboxController;


public class ElevatorControlRaw extends Command{
    ElevatorIONeo ElevatorSubsystem;
    XboxController controller;
    /**
     * This is really for testing, and a raw controller for the operator to use whenever they MUST, not recommend for use when other commands are working
     * @param Elevator
     * @param m_controller
     */
    ElevatorControlRaw(ElevatorIONeo Elevator, XboxController m_controller){
        ElevatorSubsystem = Elevator; 
        controller = m_controller;
        addRequirements(Elevator);
    }
    @Override
    public void execute(){
        ElevatorSubsystem.setVoltage(controller.getLeftX() * 12);
    }
    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
