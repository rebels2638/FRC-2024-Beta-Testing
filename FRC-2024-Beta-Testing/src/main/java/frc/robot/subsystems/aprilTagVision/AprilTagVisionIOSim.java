// package frc.robot.subsystems.aprilTagVision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.SimVisionSystem;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import frc.robot.utils.Constants;
// import java.io.IOException;

// public class AprilTagVisionIOSim implements AprilTagVisionIO {
//     // The layout of AprilTags which we want to add to the vision system
//     private AprilTagFieldLayout tagLayout;

//     private SimVisionSystem frontRightCameraSimSystem;
//     private SimVisionSystem frontLeftCameraSimSystem;
//     private SimVisionSystem backRightCameraSimSystem;
//     private SimVisionSystem backLeftCameraSimSystem;

//     private PhotonCamera frontRightCamera;
//     private PhotonCamera frontLeftCamera;
//     private PhotonCamera backRightCamera;
//     private PhotonCamera backLeftCamera;

//     {
//         try {
//             tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
//         } catch (IOException e) {
//             // Handle the exception here, e.g. print an error message or throw a custom exception
//             e.printStackTrace();
//         }
//     }

//     // A vision system sim labelled as "main" in NetworkTables
//     public AprilTagVisionIOSim() {
//         frontRightCamera = new PhotonCamera(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME);
//         frontLeftCamera = new PhotonCamera(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME);
//         backRightCamera = new PhotonCamera(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME);
//         backLeftCamera = new PhotonCamera(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME);

        
//         frontRightCameraSimSystem = new SimVisionSystem(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME, 70, Constants.VisionConstants.FRONT_RIGHT_CAMERA_ROBOT_TO_CAMERA, 100, 1000, 1000, 0);
//         frontLeftCameraSimSystem = new SimVisionSystem(Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME, 70, Constants.VisionConstants.FRONT_LEFT_CAMERA_ROBOT_TO_CAMERA, 100, 1000, 1000, 0);
//         backRightCameraSimSystem = new SimVisionSystem(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME, 70, Constants.VisionConstants.BACK_RIGHT_CAMERA_ROBOT_TO_CAMERA, 100, 1000, 1000, 0);
//         backLeftCameraSimSystem = new SimVisionSystem(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME, 70, Constants.VisionConstants.BACK_LEFT_CAMERA_ROBOT_TO_CAMERA, 100, 1000, 1000, 0);
            

//         frontRightCameraSimSystem.addVisionTargets(tagLayout);
//         frontLeftCameraSimSystem.addVisionTargets(tagLayout);
//         backRightCameraSimSystem.addVisionTargets(tagLayout);
//         backLeftCameraSimSystem.addVisionTargets(tagLayout);
//     }

//     @Override
//     public void updateInputs(AprilTagVisionIOInputs inputs, Pose2d prevEstimatedRobotPose) {
//         // Update with the simulated drivetrain pose. This should be called every loop in simulation.
//         frontRightCameraSimSystem.processFrame(prevEstimatedRobotPose);
//         frontLeftCameraSimSystem.processFrame(prevEstimatedRobotPose);
//         backRightCameraSimSystem.processFrame(prevEstimatedRobotPose);
//         backLeftCameraSimSystem.processFrame(prevEstimatedRobotPose);
//     }
// }