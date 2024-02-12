package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Utils.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RebelUtil {
    public static final double EPSILON = 1e-12;
    private final double kAdjustmentParam = 0.01;
    private final double kErrorMargin = 0.14;

    /**
     * @param toConstrain
     * @param min
     * @param max
     * @return
     */
    public static double constrain (double toConstrain, double min, double max) {
        if (toConstrain > max) {
            return max;
        }
        if (toConstrain < min) {
            return min;
        }
        return toConstrain;
    }

    public boolean driveRobotToPose(Pose2d pose) {
        this.followPathHolonomic = AutoBuilder.pathfindToPose(pose, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED));

        CommandScheduler.getInstance().schedule(followPathHolonomic);

        return true;
    }

//     public Pose2d calculateAlignedPose(Pose2d intialPose, Pose3d shooterPose, Pose3d targetPointPose) {

//         Pose2d drivebasePose = new Pose2d(initialPose.getX(),
//                                           this.shooterPose.getX()*targetPointPose.getY()/targetPointPose.getX(),
//                                           intialPose.getRotation()) // elim y-axis difference

//         Pose3d currPose = new Pose3d(intialPose)
                                          
//         // 0,0,acos(dot(target,shooter)/(mag(target)*mag(shooter))) - roll,pitch,yaw
//         // can use Rot3d and then apply as a transform to currPose and then supply transformed pose (time benefits?)

//         // this.swerveSubsystem.drive(drivebasePose.getTranslation(), drivebasePose.getRotation(), true, false); // 1st boolean might be wrong
//         // the drivebase theoretically just rotated in place to to remove the y axis
// // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//         while (Math.abs(this.shooterPose.getX()/targetPointPose.getX() - this.shooterPose.getZ()/targetPointPose.getZ()) <= this.kErrorMargin) { // z-axis difference elimination

//             if (this.shooterPose.getX()/targetPointPose.getX() > this.shooterPose.getZ()/targetPointPose.getZ()) { // might need to flip the sign idrc rn, emp. testing
//                 drivebasePose = new Pose2d(drivebasePose.getX()+this.kAdjustmentParam, drivebasePose.getY());
//             }

//             else {
//                 drivebasePose = new Pose2d(drivebasePose.getX()-this.kAdjustmentParam, drivebasePose.getY());
//             }

//             targetPointPose = this.visionSubsystem.getValidShotPoint().relativeTo(new Pose3d(this.swerveSubsystem.getPose())); // doesn't exist... yet
//         }

//         return drivebasePose;

//     }

}