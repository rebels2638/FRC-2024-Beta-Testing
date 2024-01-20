package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.input.XboxController;
 

public class IntakeToggle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final double m_goalVelocity; // rotations per second
  
  public IntakeToggle(Intake intakeSubsystem, double velocity) {
    m_intake = intakeSubsystem;
    m_goalVelocity = velocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

 // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setVelocitySetpoint(m_goalVelocity);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { }   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
