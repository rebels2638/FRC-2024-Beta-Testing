package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.lib.RebelUtil;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANDrivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CANDrivetrain m_driveSubsystem;
  private final XboxController xboxDriver;
  private final double MAX_FORWARD_SPEED = 10;
  private final double MAX_TURN_SPEED = 10;
  private final SlewRateLimiter rateLimiter;
  private double MAX_FORWARD_ACCEL = 7;
  private double MAX_BACKWARD_ACCEL = -7;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(CANDrivetrain driveSubsystem, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    rateLimiter = new SlewRateLimiter(MAX_FORWARD_ACCEL, MAX_BACKWARD_ACCEL, 0);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive_power = xboxDriver.getLeftY();
    double turn_power = xboxDriver.getRightX();
  
    // drive_power = rateLimiter.calculate(drive_power);
    m_driveSubsystem.drive(drive_power - turn_power, drive_power + turn_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}