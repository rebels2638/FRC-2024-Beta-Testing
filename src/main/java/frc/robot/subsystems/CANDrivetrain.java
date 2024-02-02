// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.LauncherConstants.kTrackWidth;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  // public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  WPI_VictorSPX leftFront = new WPI_VictorSPX(kLeftFrontID);
  WPI_VictorSPX leftRear = new WPI_VictorSPX(kLeftRearID);
  WPI_VictorSPX rightFront = new WPI_VictorSPX(kRightFrontID);
  WPI_VictorSPX rightRear = new WPI_VictorSPX(kRightRearID);
  // private final SimpleMotorFeedforward m_feedforwardLeft = new SimpleMotorFeedforward(1, 3);
  // private final SimpleMotorFeedforward m_feedforwardRight = new SimpleMotorFeedforward(1, 3);

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {

    // leftFront.configFactoryDefault();
    // leftRear.configFactoryDefault();
    // rightFront.configFactoryDefault();
    // rightRear.configFactoryDefault();

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    rightFront.setInverted(true);
    rightRear.setInverted(true);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  // public void drive(double input, double angVelo) {
  //   var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(input, 0.0, angVelo));
  //   // final double leftFeedforward = m_feedforwardLeft.calculate(wheelSpeeds.leftMetersPerSecond);
  //   // final double rightFeedforward = m_feedforwardRight.calculate(wheelSpeeds.rightMetersPerSecond);

  //   leftFront.set(VictorSPXControlMode.Velocity, wheelSpeeds.leftMetersPerSecond);
  //   rightFront.set(VictorSPXControlMode.Velocity, wheelSpeeds.rightMetersPerSecond);

  //   // leftFront.set(VictorSPXControlMode.PercentOutput, input);
  //   // rightFront.set(VictorSPXControlMode.PercentOutput, in);
  // }

  public void drive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  public void stop() {
    drive(0, 0);
  }

  // public void drive(double xSpeed, double rot) {
    
  // }


  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
