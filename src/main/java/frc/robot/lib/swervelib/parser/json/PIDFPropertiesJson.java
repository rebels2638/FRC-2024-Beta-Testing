package frc.robot.lib.swervelib.parser.json;

import frc.robot.lib.swervelib.parser.PIDFConfig;

/**
 * {@link frc.robot.lib.swervelib.SwerveModule} PID with Feedforward for the drive motor and angle motor.
 */
public class PIDFPropertiesJson
{

  /**
   * The PIDF with Integral Zone used for the drive motor.
   */
  public PIDFConfig drive;
  /**
   * The PIDF with Integral Zone used for the angle motor.
   */
  public PIDFConfig angle;

  public double ANGLE_FF;
}
