// package frc.robot.subsystems.aprilTagVision;


// import org.littletonrobotics.junction.Logger;
// import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AprilTagVision extends SubsystemBase{
//     private AprilTagVisionIO io;
//     private Pose2d prevEstimatedRobotPose = new Pose2d();
//     AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
//     double timestampSeconds = 0;
//     public AprilTagVision(AprilTagVisionIO io) {
//         this.io = io;
//     }

//     @Override
//     public void periodic() {
//         io.updateInputs(inputs, prevEstimatedRobotPose);
//         Logger.getInstance().processInputs("Pivot", inputs);
 
//     }

//     public Pose2d getEstimatedRobotPose(Pose2d prevEstimatedRobotPose) {
//         this.prevEstimatedRobotPose = prevEstimatedRobotPose;

//         // take the average of all poses that exist from the last input update
//         double xAverage = 0;
//         double yAverage = 0;
//         double rotAverage = 0;
//         timestampSeconds = 0;
//         int numPoses = 0;
//         if (inputs.backRightEstimatedPose != null) {
//             EstimatedRobotPose backRightEstimatedPose = inputs.backRightEstimatedPose.get();
//             if (inputs.backRightEstimatedPose.isPresent()) {
//                 xAverage += backRightEstimatedPose.estimatedPose.getX();
//                 yAverage += backRightEstimatedPose.estimatedPose.getY();
//                 rotAverage += backRightEstimatedPose.estimatedPose.getRotation().getAngle();
//                 numPoses++;

//                 timestampSeconds = backRightEstimatedPose.timestampSeconds;
//             }
//         }
//         if (inputs.backLeftEstimatedPose != null) {
//             EstimatedRobotPose backLeftEstimatedPose = inputs.backLeftEstimatedPose.get();
//             if (backLeftEstimatedPose != null) {
//                 xAverage += backLeftEstimatedPose.estimatedPose.getX();
//                 yAverage += backLeftEstimatedPose.estimatedPose.getY();
//                 rotAverage += backLeftEstimatedPose.estimatedPose.getRotation().getAngle();
//                 numPoses++;

//                 timestampSeconds = backLeftEstimatedPose.timestampSeconds;
//             }
//         }
//         if (inputs.frontRightEstimatedPose != null) {
//             EstimatedRobotPose frontRightEstimatedPose = inputs.frontRightEstimatedPose.get();
//             if (frontRightEstimatedPose != null) {
//                 xAverage += frontRightEstimatedPose.estimatedPose.getX();
//                 yAverage += frontRightEstimatedPose.estimatedPose.getY();
//                 rotAverage += frontRightEstimatedPose.estimatedPose.getRotation().getAngle();
//                 numPoses++;

//                 timestampSeconds = frontRightEstimatedPose.timestampSeconds;
//             }
//         }
//         if (inputs.frontLeftEstimatedPose != null) {
//             EstimatedRobotPose frontLeftEstimatedPose = inputs.frontLeftEstimatedPose.get();
//             if (frontLeftEstimatedPose != null) {
//                 xAverage += frontLeftEstimatedPose.estimatedPose.getX();
//                 yAverage += frontLeftEstimatedPose.estimatedPose.getY();
//                 rotAverage += frontLeftEstimatedPose.estimatedPose.getRotation().getAngle();
//                 numPoses++;

//                 timestampSeconds = frontLeftEstimatedPose.timestampSeconds;
//             }
//         }

//         if (numPoses == 0) {
//             timestampSeconds = Timer.getFPGATimestamp();
//             return prevEstimatedRobotPose;
//         }
//         timestampSeconds /= numPoses;
//         return new Pose2d(xAverage/numPoses, yAverage/numPoses, new Rotation2d(rotAverage/numPoses));
//     }

//     public double getTimestampSeconds() {
//         return timestampSeconds;
//     }
// }
