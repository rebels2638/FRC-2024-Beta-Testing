// package frc.robot.subsystems.aprilTagVision;

// import java.util.Optional;

// import org.littletonrobotics.junction.AutoLog;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Pose2d;

// public interface AprilTagVisionIO {
//     @AutoLog
//     public static class AprilTagVisionIOInputs {
//         public PhotonTrackedTarget frontRightBestTarget;
//         public PhotonTrackedTarget frontLeftBestTarget;
//         public PhotonTrackedTarget backRightBestTarget;
//         public PhotonTrackedTarget backLeftBestTarget;

//         public double frontRightPipleineLatency;
//         public double frontLeftPipleineLatency;
//         public double backRightPipleineLatency;
//         public double backLeftPipleineLatency;

//         public double frontRightTimestamp;
//         public double frontLeftTimestamp;
//         public double backRightTimestamp;
//         public double backLeftTimestamp;

//         public Optional<EstimatedRobotPose> frontRightEstimatedPose;
//         public Optional<EstimatedRobotPose> frontLeftEstimatedPose;
//         public Optional<EstimatedRobotPose> backRightEstimatedPose;
//         public Optional<EstimatedRobotPose> backLeftEstimatedPose;
//     }

//     public default void updateInputs(AprilTagVisionIOInputs inputs, Pose2d refrencePose) {
//     }
// }
