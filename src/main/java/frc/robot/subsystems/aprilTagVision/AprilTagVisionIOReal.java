// package frc.robot.subsystems.aprilTagVision;

// import java.io.IOException;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import frc.robot.utils.Constants;

// public class AprilTagVisionIOReal implements AprilTagVisionIO {
//     private PhotonCamera frontRightCamera;
//     private PhotonCamera frontLeftCamera;
//     private PhotonCamera backRightCamera;
//     private PhotonCamera backLeftCamera;
//     // The parameter for loadFromResource() will be different depending on the game.
//     AprilTagFieldLayout aprilTagFieldLayout;

//     {
//         try {
//             aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
//         } catch (IOException e) {
//             // Handle the exception here, e.g. print an error message or throw a custom exception
//             e.printStackTrace();
//         }
//     }


//     // Construct PhotonPoseEstimator
//     PhotonPoseEstimator frontRightPhotonPoseEstimator;
//     PhotonPoseEstimator frontLeftPhotonPoseEstimator;
//     PhotonPoseEstimator backRightPhotonPoseEstimator;
//     PhotonPoseEstimator backLeftPhotonPoseEstimator;

//     public AprilTagVisionIOReal() {
//         frontRightCamera = new PhotonCamera(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME);
//         Transform3d frontRightCameraRobotToCam = Constants.VisionConstants.FRONT_RIGHT_CAMERA_ROBOT_TO_CAMERA;

//         frontLeftCamera = new PhotonCamera(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME);
//         Transform3d frontLeftCameraRobotToCam = Constants.VisionConstants.FRONT_LEFT_CAMERA_ROBOT_TO_CAMERA;

//         backRightCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);
//         Transform3d backRightCameraRobotToCam = Constants.VisionConstants.BACK_RIGHT_CAMERA_ROBOT_TO_CAMERA;
        
//         backLeftCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);
//         Transform3d backLeftCameraRobotToCam = Constants.VisionConstants.BACK_LEFT_CAMERA_ROBOT_TO_CAMERA;

//         // make photon pose estimators for all cams
//         //TODO: change to MULTI_TAG_PNP_ON_COPROCESSOR when it comes out!!!!
//         frontRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
//                                                         frontRightCamera, frontRightCameraRobotToCam);
        
//         frontLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
//                                                         frontLeftCamera, frontLeftCameraRobotToCam);
        
//         backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
//                                                         backRightCamera, backRightCameraRobotToCam);
        
//         backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY,
//                                                         backLeftCamera, backLeftCameraRobotToCam);


//     }
    
//     @Override
//     public void updateInputs(AprilTagVisionIOInputs inputs, Pose2d prevEstimatedRobotPose) {
        
//         inputs.frontRightBestTarget = frontRightCamera.getLatestResult().getBestTarget();
//         inputs.frontRightPipleineLatency = frontRightCamera.getLatestResult().getLatencyMillis();

//         inputs.backLeftBestTarget = backLeftCamera.getLatestResult().getBestTarget();
//         inputs.backLeftPipleineLatency = backLeftCamera.getLatestResult().getLatencyMillis();

//         inputs.frontLeftBestTarget = frontLeftCamera.getLatestResult().getBestTarget();
//         inputs.frontLeftPipleineLatency = frontLeftCamera.getLatestResult().getLatencyMillis();

//         inputs.backRightBestTarget = backRightCamera.getLatestResult().getBestTarget();
//         inputs.backRightPipleineLatency = backRightCamera.getLatestResult().getLatencyMillis();
        
//         frontRightPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         backLeftPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         frontLeftPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         backRightPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

//         Optional<EstimatedRobotPose> frontRightEstimatedPoseOptional = frontRightPhotonPoseEstimator.update();
//         if (frontRightEstimatedPoseOptional.isPresent()) {
//             inputs.frontRightEstimatedPose = frontRightEstimatedPoseOptional;
//             inputs.frontRightTimestamp = frontRightCamera.getLatestResult().getTimestampSeconds();
//         }

//         Optional<EstimatedRobotPose> backLeftEstimatedPoseOptional = backLeftPhotonPoseEstimator.update();
//         if (backLeftEstimatedPoseOptional.isPresent()) {
//             inputs.backLeftEstimatedPose = backLeftEstimatedPoseOptional;
//             inputs.backLeftTimestamp = backLeftCamera.getLatestResult().getTimestampSeconds();
//         }

//         Optional<EstimatedRobotPose> frontLeftEstimatedPoseOptional = frontLeftPhotonPoseEstimator.update();
//         if (frontLeftEstimatedPoseOptional.isPresent()) {
//             inputs.frontLeftEstimatedPose = frontLeftEstimatedPoseOptional;
//             inputs.frontLeftTimestamp = frontLeftCamera.getLatestResult().getTimestampSeconds();
//         }

//         Optional<EstimatedRobotPose> backRightEstimatedPoseOptional = backRightPhotonPoseEstimator.update();
//         if (backRightEstimatedPoseOptional.isPresent()) {
//             inputs.backRightEstimatedPose = backRightEstimatedPoseOptional;
//             inputs.backRightTimestamp = backRightCamera.getLatestResult().getTimestampSeconds();
//         }
//     }
// }