// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;

// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.aprilTagVision.AprilTagVision;

// public class AlignWithTargetPoint extends Command {
//     private final Shooter shooterSubsystem;
//     private final AprilTagVision visionSubsystem;
//     private final Pose3d shooterPose;
//     private final SwerveSubsystem swerveSubsystem;
//     private final double kAdjustmentParam;

//     public AlignWithTargetPoint(Shooter shooterSubsystem, AprilTagVision visionSubsystem, SwerveSubsystem swerveSubsystem) {
//         this.shooterSubsystem = shooterSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.swerveSubsystem = swerveSubsystem;
//         this.shooterPose = new Pose3d(0,0,0,new Rotation3d(0,0,0)); // first three zeroes are real measurments, no euler angles (rot. at tip)
//         this.kAdjustmentParam = 0.01; // in meters?
//         addRequirements(shooterSubsystem);
//         addRequirements(visionSubsystem);
//         addRequirements(swerveSubsystem);
//     }

//     @override
//     public void execute() {
//         Pose3d initialPose = new Pose3d(this.swerveSubsystem.getPose()); // assume that everything in x,y,z??
//         Pose3d targetPointPose = this.visionSubsystem.getValidShotPoint().relativeTo(currPose); // getValidShotPoint() is the whole LL3 quad detection thing... WIP
        
//         Pose2d drivebasePose = new Pose2d(initialPose.getX(),
//                                           this.shooterPose.getX()*targetPointPose.getY()/targetPointPose.getX()
//                                           ) // elim y-axis difference
                                          
//         // 0,0,acos(dot(target,shooter)/(mag(target)*mag(shooter))) - roll,pitch,yaw
//         // can use Rot3d and then apply as a transform to currPose and then supply transformed pose (time benefits?)

//         this.swerveSubsystem.drive(drivebasePose.getTranslation(), drivebasePose.getRotation(), true, false); // 1st boolean might be wrong
//         // the drivebase theoretically just rotated in place to to remove the y axis
// // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//         while (this.shooterPose.getX()/targetPointPose.getX() != this.shooterPose.getZ()/targetPointPose.getZ()) { // z-axis difference elimination

//             if (this.shooterPose.getX()/targetPointPose.getX() > this.shooterPose.getZ()/targetPointPose.getZ()) { // might need to flip the sign idrc rn, emp. testing
//                 drivebasePose = new Pose2d(drivebasePose.getX()+this.kAdjustmentParam, drivebasePose.getY());
//             }
//             else {
//                 drivebasePose = new Pose2d(drivebasePose.getX()-this.kAdjustmentParam, drivebasePose.getY());
//             }

//             this.swerveSubsystem.drive(drivebasePose.getTranslation(), drivebasePose.getRotation(), true, false)
//             targetPointPose = this.visionSubsystem.getValidShotPoint().relativeTo(new Pose3d(this.swerveSubsystem.getPose())); // doesn't exist... yet
//         }

//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }