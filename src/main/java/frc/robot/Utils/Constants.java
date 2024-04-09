// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

// import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final Mode currentMode = Mode.REAL; // TODO: change this if sim

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static final double ROBOT_MASS = (60) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(0)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.2; //s, 20ms + 110ms sprk max velocity lag

  public static final class Telop {
    public static final double MAX_SPEED_METERS_PER_SEC = 9;
    //public static final double MAX_ROT 
  }
  public static final class Auton
  {

  public static final PIDConstants TRANSLATION_PID_CONFIG = new PIDConstants(1, 0, 0.0001); //1,  0.0002
    public static final PIDConstants ANGLE_PID_CONFIG = new PIDConstants(3.855, 0, 0.00008); //3.855, 0 ,0.00012

    public static final double MAX_SPEED = 4;
    public static final double MAX_ACCELERATION = 2;
    public static final double MAX_ANGULAR_VELO_RPS = 2; // 1
    public static final double MAX_ANGULAR_ACCEL_RPS_SQUARED = 1; // 0.5

    public static final HolonomicPathFollowerConfig DRIVE_CONTROLLER_CONFIG = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        Constants.Auton.TRANSLATION_PID_CONFIG, // Translation PID constants
                        Constants.Auton.ANGLE_PID_CONFIG, // Rotation PID constants
                        Constants.Auton.MAX_SPEED, // Max module speed, in m/s
                        // I LOVE THE PYTHAGPREAN THEOREM
                        0.5384061785684372, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

  }
  public static final class Drivebase
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 1; // seconds
    public static final double MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC = 12; //11.5 // 9
    public static final double MAX_DEG_SEC_ROTATIONAL_VELOCITY = 235;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    // yes, this high
    public static final double LEFT_X_DEADBAND = 0.09;
    public static final double LEFT_Y_DEADBAND = 0.09;
    
    public static final double RIGHT_X_DEADBAND = 0.09;
  }

  public static final class FieldConstants {
    public static final Translation2d[] autoAlignTranslationArr = 
      { new Translation2d(1.77, 0.50),
      new Translation2d(1.77, 1.07),
      new Translation2d(1.77, 1.62),
      new Translation2d(1.77, 2.18),
      new Translation2d(1.77, 2.74),
      new Translation2d(1.77, 3.30),
      new Translation2d(1.77, 3.85),
      new Translation2d(1.77, 4.42),
      new Translation2d(1.77, 4.97) };
    public static final Pose2d autoAlignAMPPose = new Pose2d(new Translation2d(1.82, 7.33), new Rotation2d(Math.toRadians(90)));
    // public static final AprilTagFieldLayout aprilTagFieldLayout 
    // = new AprilTagFieldLayout();

    public static final Pose2d[] autoAlignTrapPose = { new Pose2d(new Translation2d(4.31, 3.13), new Rotation2d(Math.toRadians(60))), 
                                                       new Pose2d(new Translation2d(4.23, 5.18), new Rotation2d(Math.toRadians(-60.00))),
                                                       new Pose2d(new Translation2d(6.08, 4.00), new Rotation2d(Math.toRadians(180.00))),
};
    
  }
  public static final class VisionConstants {
    // this is photon stuff:
    // public static final String FRONT_RIGHT_CAMERA_NAME = "frontRightCamera";
    // public static final String FRONT_LEFT_CAMERA_NAME = "frontLeftCamera";
    // public static final String BACK_RIGHT_CAMERA_NAME = "backRightCamera";
    // public static final String BACK_LEFT_CAMERA_NAME = "backLeftCamera";
    
    // public static final double FRONT_RIGHT_CAMERA_FOV_DIAGONAL = 90;
    // public static final Transform3d FRONT_RIGHT_CAMERA_ROBOT_TO_CAMERA
    //  = new Transform3d(new Translation3d(.33, .33, 0), new Rotation3d(0, -0.59, -0.79));
    // public static final double FRONT_RIGHT_MAX_LED_RANGE_METERS = 100;
    // public static final int FRONT_RIGHT_CAMERA_RESOLUTION_WIDTH = 1280;
    // public static final int FRONT_RIGHT_CAMERA_RESOLUTION_HEIGHT = 720;
    // public static final double FRONT_RIGHT_CAMERA_MIN_TARGET_AREA = 0.05;

    // public static final double FRONT_LEFT_CAMERA_FOV_DIAGONAL = 90;
    // public static final Transform3d FRONT_LEFT_CAMERA_ROBOT_TO_CAMERA 
    // = new Transform3d(new Translation3d(-.33, .33, 0), new Rotation3d(0, -0.59, 0.79));
    // public static final double FRONT_LEFT_MAX_LED_RANGE_METERS = 100;
    // public static final int FRONT_LEFT_CAMERA_RESOLUTION_WIDTH = 1280;
    // public static final int FRONT_LEFT_CAMERA_RESOLUTION_HEIGHT = 720;
    // public static final double FRONT_LEFT_CAMERA_MIN_TARGET_AREA = 0.05;

    // public static final double BACK_RIGHT_CAMERA_FOV_DIAGONAL = 90;
    // public static final Transform3d BACK_RIGHT_CAMERA_ROBOT_TO_CAMERA 
    // = new Transform3d(new Translation3d(.33, -.33, 0), new Rotation3d(0, 0.59, -0.79 + Math.PI));
    // public static final double BACK_RIGHT_MAX_LED_RANGE_METERS = 100;
    // public static final int BACK_RIGHT_CAMERA_RESOLUTION_WIDTH = 1280;
    // public static final int BACK_RIGHT_CAMERA_RESOLUTION_HEIGHT = 720;
    // public static final double BACK_RIGHT_CAMERA_MIN_TARGET_AREA = 0.05;

    // public static final double BACK_LEFT_CAMERA_FOV_DIAGONAL = 90;
    // public static final Transform3d BACK_LEFT_CAMERA_ROBOT_TO_CAMERA 
    // = new Transform3d(new Translation3d(-.33, -.33, 0), new Rotation3d(0, 0.59, 0.79 + Math.PI));
    // public static final double BACK_LEFT_MAX_LED_RANGE_METERS = 100;
    // public static final int BACK_LEFT_CAMERA_RESOLUTION_WIDTH = 1280;
    // public static final int BACK_LEFT_CAMERA_RESOLUTION_HEIGHT = 720;
    // public static final double BACK_LEFT_CAMERA_MIN_TARGET_AREA = 0.05;

    public static AprilTagFieldLayout aprilTagFieldLayout;
    public VisionConstants() {
      // The parameter for loadFromResource() will be different depending on the game.
      try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}