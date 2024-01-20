package frc.robot.lib.swervelib.parser.json;

import frc.robot.lib.swervelib.parser.PIDFConfig;
import frc.robot.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.robot.lib.swervelib.parser.SwerveDriveConfiguration;

/**
 * {@link frc.robot.lib.swervelib.SwerveController} parsed class. Used to access the JSON data.
 */
public class ControllerPropertiesJson
{

  /**
   * The minimum radius of the angle control joystick to allow for heading adjustment of the robot.
   */
  public double     angleJoystickRadiusDeadband;
  /**
   * The PID used to control the robot heading.
   */
  public PIDFConfig heading;

  /**
   * Create the {@link SwerveControllerConfiguration} based on parsed and given data.
   *
   * @param driveConfiguration {@link SwerveDriveConfiguration} parsed configuration.
   * @param maxSpeedMPS        Maximum speed in meters per second for the angular acceleration of the robot.
   * @return {@link SwerveControllerConfiguration} object based on parsed data.
   */
  public SwerveControllerConfiguration createControllerConfiguration(
      SwerveDriveConfiguration driveConfiguration, double maxSpeedMPS)
  {
    return new SwerveControllerConfiguration(
        driveConfiguration, heading, angleJoystickRadiusDeadband, maxSpeedMPS);
  }
}
