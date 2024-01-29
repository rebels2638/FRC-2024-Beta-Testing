// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.SerialPort.Port;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.kauailabs.navx.frc.AHRS;
// // import org.photonvision.EstimatedRobotPose;
// // import frc.robot.commands.PoseEstimation;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import java.util.Optional;
// import static frc.robot.Constants.DrivetrainConstants.*;
// // import org.photonvision.*;

// /** Represents a differential drive style drivetrain. */
// public class Drivetrain extends SubsystemBase {
//   public static final double kMaxSpeed = 3.0 * 0.25; // meters per second
//   public static final double kMaxAngularSpeed = 2 * Math.PI * 0.25; // one rotation per second

//   private static final double kTrackWidth = 0.381 * 2; // meters
//   private static final double kWheelRadius = 0.0508; // meters
//   private static final int kEncoderResolution = 4096;

//   private final MotorController m_leftLeader = new WPI_VictorSPX(kLeftRearID);
//   private final MotorController m_leftFollower = new WPI_VictorSPX(kLeftFrontID);
//   private final MotorController m_rightLeader = new WPI_VictorSPX(kRightRearID);
//   private final MotorController m_rightFollower = new WPI_VictorSPX(kRightFrontID);

//   public final Encoder m_leftEncoder = new Encoder(5, 6);
//   public final Encoder m_rightEncoder = new Encoder(7, 8);

//   private final MotorControllerGroup m_leftGroup =
//       new MotorControllerGroup(m_leftLeader, m_leftFollower);
//   private final MotorControllerGroup m_rightGroup =
//       new MotorControllerGroup(m_rightLeader, m_rightFollower);

//   private final AHRS m_gyro = new AHRS(Port.kUSB);

//   private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
//   private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

//   public final DifferentialDriveKinematics m_kinematics =
//       new DifferentialDriveKinematics(kTrackWidth);

//   public final DifferentialDriveOdometry m_odometry;

//   // Gains are for example purposes only - must be determined for your own robot!
//   private final SimpleMotorFeedforward m_feedforwardLeft = new SimpleMotorFeedforward(1, 3);
//   private final SimpleMotorFeedforward m_feedforwardRight = new SimpleMotorFeedforward(1, 3);

//   // public PoseEstimation pcw;

//   private final DifferentialDrivePoseEstimator m_poseEstimator =
//   new DifferentialDrivePoseEstimator(
//           m_kinematics, m_gyro.getRotation2d(), 0.0, 0.0, new Pose2d());


//   /**
//    * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
//    * gyro.
//    */
//   public Drivetrain() {
//     m_gyro.reset();
//     // pcw = new PoseEstimation();
 
//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     m_rightGroup.setInverted(true); // changed back to true
//     //m_leftGroup.setInverted(true);
//     // Set the distance per pulse for the drive encoders. We can simply use the
//     // distance traveled for one rotation of the wheel divided by the encoder
//     // resolution.
//     m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
//     m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

//     m_leftEncoder.reset();
//     m_rightEncoder.reset();

//     m_odometry =
//         new DifferentialDriveOdometry(
//             m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

//     Shuffleboard.getTab("Drive").add(new InstantCommand(() -> m_gyro.reset()));
//   }

//   /**
//    * Sets the desired wheel speeds.
//    *
//    * @param speeds The desired wheel speeds.
//    */
//   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
//     final double leftFeedforward = m_feedforwardLeft.calculate(speeds.leftMetersPerSecond);
//     final double rightFeedforward = m_feedforwardRight.calculate(speeds.rightMetersPerSecond);

//     final double leftOutput =
//         m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
//     final double rightOutput =
//         m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
//     m_leftGroup.setVoltage(leftOutput + leftFeedforward);
//     m_rightGroup.setVoltage(rightOutput + rightFeedforward);
 
//     // System.out.println("left: " + leftFeedforward + " right: " + rightFeedforward);
//   }

//   /**
//    * Drives the robot with the given linear velocity and angular velocity.
//    *
//    * @param xSpeed Linear velocity in m/s.
//    * @param rot Angular velocity in rad/s.
//    */
//   public void drive(double xSpeed, double rot) {
//     var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
//     setSpeeds(wheelSpeeds);
//   }

//   /** Updates the field-relative position. */
//   public void updateOdometry() {
//     m_odometry.update(
//         m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

//         // Optional<EstimatedRobotPose> result =
//         // pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

//   }

//   public Rotation2d getRotation2d() {
//     return m_gyro.getRotation2d();
//   }
// }