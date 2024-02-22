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
        
        double height = this.ElevatorSubsystem.getShooterHeightMeters();
        if (controller.getLeftY() > 0.04 || controller.getLeftY() < 0.04) {
            height += controller.getLeftY()*0.54;
        //     if (controller.getLeftY() > 0) {
        //         height = 0.54-controller.getLeftY()*0.54;
        //     }
        //     if (controller.getLeftY() == 0) {
        //         height = this.ElevatorSubsystem.getShooterHeightMeters();
        //     }
        //     else {
        //         height = -controller.getLeftY()*0.54;
        //     }
        }
        ElevatorSubsystem.setHeightMeters(height);
    }

    
    @Override
    //This is a default command, it should never finish theoretically
    public boolean isFinished(){
        return false;
    }

    
}
