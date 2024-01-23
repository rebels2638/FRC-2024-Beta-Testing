package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.lib.input.RebelUtil;
import frc.robot.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterController extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final XboxController s_controller; // s_controller is shooter’s controller
  
  public ShooterController(Shooter shooterSubsystem, XboxController controller) {
    s_controller = controller;
    m_shooter = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	System.out.println("Shooter Initialized");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredVelo = s_controller.getLeftY() * Shooter.kMaxSpeed;
    m_shooter.setVelocitySetpoint(desiredVelo);
    System.out.println("Controller: " + s_controller.getLeftY());
  }   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
