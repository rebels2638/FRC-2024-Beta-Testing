// package frc.robot.commands.shooter;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;

// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.aprilTagVision.AprilTagVision;

// import frc.robot.Utils.RebelUtil;

// public class AlignWithTargetPoint extends Command {
//     private final Shooter shooterSubsystem;
//     private final Pose3d shooterPose;
//     private final SwerveSubsystem swerveSubsystem;

//     public AlignWithTargetPoint(Shooter shooterSubsystem, SwerveSubsystem swerveSubsystem) {
//         this.shooterSubsystem = shooterSubsystem;
//         this.swerveSubsystem = swerveSubsystem;
//         this.shooterPose = new Pose3d(0,0,0,new Rotation3d(0,0,0)); // first three zeroes are real measurments, no euler angles (rot. at tip)
//         addRequirements(shooterSubsystem);
//         addRequirements(swerveSubsystem);
//     }

//     @override
//     public void execute() {
//         Pose3d initialPose = this.swerveSubsystem.getPose(); // assume that everything in x,y,z??
//         Pose3d targetPointPose = this.visionSubsystem.getValidShotPoint().relativeTo(initialPose);

//         Pose2d computedPose = RebelUtil.calculateAlignedPose(initialPose, this.shooterPose, targetPointPose);
//         RebelUtil.driveRobotToPose(computedPose);
        
//         this.shooterSubsystem.setFlywheelVelocity(0.0); // actually implement physics for linear freefall to calc v_angular

//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }

// }