package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.Utils.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.intake.IntakeIO;
// import frc.robot.subsystems.limelight.PoseLimelight;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RebelUtil {
    public static final double EPSILON = 1e-12;
    public static double flywheelSpeed = 0.0;
    private static final double kAdjustmentParam = 0.050761421; // its like 2in
    private static final double kErrorMargin = 0.0761421; // its like 3in

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

    public static boolean driveRobotToPose(Pose2d pose) {
        Command followPathHolonomic = AutoBuilder.pathfindToPose(pose, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED));

        CommandScheduler.getInstance().schedule(followPathHolonomic);

        return true;
    }

    public static double dot(Pose3d a, Pose3d b) {
        return (a.getX()*b.getX()) + (a.getY()*b.getY()) + (a.getZ()*b.getZ());
    }
    
    public static double mag(Pose3d a) {
        return Math.sqrt(Math.pow(a.getX(), 2) + Math.pow(a.getY(), 2) + Math.pow(a.getZ(), 2));
    }

//     public static Pose2d calculateAlignedPose(PoseLimelight visionSubsystem, Pose2d initialPose, Pose3d shooterPose, Pose3d targetPointPose) {

//         Pose3d s_aligned = new Pose3d(shooterPose.getX(),
//                                       shooterPose.getX()*targetPointPose.getY()/targetPointPose.getX(),
//                                       shooterPose.getZ(),
//                                       new Rotation3d(0,0,0)); // elim y-axis difference

//         Rotation3d rot = new Rotation3d(0,0, Math.acos(dot(targetPointPose, s_aligned)/(mag(targetPointPose)*mag(s_aligned))));
//         Pose3d intermediate = new Pose3d(initialPose).rotateBy(rot);
//         Pose2d drivebasePose = new Pose2d(initialPose.getTranslation(), initialPose.getRotation().plus(new Rotation2d(intermediate.getRotation().getZ())));
                                          
//         // 0,0,acos(dot(target,shooter)/(mag(target)*mag(shooter))) - roll,pitch,yaw
//         // can use Rot3d and then apply as a transform to currPose and then supply transformed pose (time benefits?)

//         // this.swerveSubsystem.drive(drivebasePose.getTranslation(), drivebasePose.getRotation(), true, false); // 1st boolean might be wrong
//         // the drivebase theoretically just rotated in place to to remove the y axis
// // ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//         while (Math.abs(shooterPose.getX()/targetPointPose.getX() - shooterPose.getZ()/targetPointPose.getZ()) <= kErrorMargin) { // z-axis difference elimination

//             if (shooterPose.getX()/targetPointPose.getX() > shooterPose.getZ()/targetPointPose.getZ()) { // might need to flip the sign idrc rn, emp. testing
//                 drivebasePose = new Pose2d(drivebasePose.getX()+kAdjustmentParam, drivebasePose.getY(), new Rotation2d(0));
//             }

//             else {
//                 drivebasePose = new Pose2d(drivebasePose.getX()-kAdjustmentParam, drivebasePose.getY(), new Rotation2d(0));
//             }

//             targetPointPose = visionSubsystem.getValidShotPoint().relativeTo(new Pose3d(drivebasePose));
//         }

//         return drivebasePose;

//     }

}