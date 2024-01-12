package frc.robot.lib.swervelib.parser.json;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.swervelib.encoders.SwerveAbsoluteEncoder;
import frc.robot.lib.swervelib.motors.SwerveMotor;
import frc.robot.lib.swervelib.parser.PIDFConfig;
import frc.robot.lib.swervelib.parser.SwerveModuleConfiguration;
import frc.robot.lib.swervelib.parser.SwerveModulePhysicalCharacteristics;
import frc.robot.lib.swervelib.parser.json.modules.BoolMotorJson;
import frc.robot.lib.swervelib.parser.json.modules.LocationJson;

/**
 * {@link swervelib.SwerveModule} JSON parsed class. Used to access the JSON data.
 */
public class ModuleJson
{

  /**
   * Drive motor device configuration.
   */
  public DeviceJson    drive;
  /**
   * Angle motor device configuration.
   */
  public DeviceJson    angle;
  /**
   * Absolute encoder device configuration.
   */
  public DeviceJson    encoder;
  /**
   * Defines which motors are inverted.
   */
  public BoolMotorJson inverted;
  /**
   * Absolute encoder offset from 0 in degrees.
   */
  public double        absoluteEncoderOffset;
  /**
   * Absolute encoder inversion state.
   */
  public boolean       absoluteEncoderInverted        = false;
  /**
   * The angle encoder pulse per revolution override. 1 for Neo encoder. 2048 for Falcons.
   */
  public double        angleEncoderPulsePerRevolution = 0;
  /**
   * The location of the swerve module from the center of the robot in inches.
   */
  public LocationJson  location;

  /**
   * Create the swerve module configuration based off of parsed data.
   *
   * @param anglePIDF               The PIDF values for the angle motor.
   * @param velocityPIDF            The velocity PIDF values for the drive motor.
   * @param maxSpeed                The maximum speed of the robot in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   * @param name                    Module json filename.
   * @return {@link SwerveModuleConfiguration} based on the provided data and parsed data.
   */
  public SwerveModuleConfiguration createModuleConfiguration(
      PIDFConfig anglePIDF,
      PIDFConfig velocityPIDF,
      double maxSpeed,
      SwerveModulePhysicalCharacteristics physicalCharacteristics,
      String name)
  {
    SwerveMotor           angleMotor = angle.createMotor(false);
    SwerveAbsoluteEncoder absEncoder = encoder.createEncoder();

    // If the absolute encoder is attached.
    if (absEncoder == null)
    {
      absEncoder = angle.createIntegratedEncoder(angleMotor);
      angleMotor.setAbsoluteEncoder(absEncoder);
    }

    return new SwerveModuleConfiguration(
        drive.createMotor(true),
        angleMotor,
        absEncoder,
        absoluteEncoderOffset,
        Units.inchesToMeters(Math.round(location.x) == 0 ? location.front : location.x),
        Units.inchesToMeters(Math.round(location.y) == 0 ? location.left : location.y),
        anglePIDF,
        velocityPIDF,
        maxSpeed,
        physicalCharacteristics,
        absoluteEncoderInverted,
        inverted.drive,
        inverted.angle,
        angleEncoderPulsePerRevolution == 0 ? physicalCharacteristics.angleEncoderPulsePerRotation
                                            : angleEncoderPulsePerRevolution,
        name.replaceAll("\\.json", ""));
  }
}
