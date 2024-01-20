package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignAMP extends Command {
    private Command followPathHolonomic;
    public AutoAlignAMP(SwerveSubsystem swerveSubsystem) {
        // // Create a list of bezier points from poses. Each pose represents one waypoint.
        // // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        //     new Pose2d(swerveSubsystem.getPose().getTranslation(), 
        //         new Rotation2d(Math.atan2(Constants.FeildConstants.autoAlightAMPPose.getY() - swerveSubsystem.getPose().getTranslation().getY(), 
        //         Constants.FeildConstants.autoAlightAMPPose.getX() - swerveSubsystem.getPose().getTranslation().getX()) + Math.toRadians(90))),
        //     Constants.FeildConstants.autoAlightAMPPose
        // );

        // // Create the path using the bezier points created above
        // PathPlannerPath path = new PathPlannerPath(
        //         bezierPoints,
        //         new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION,
        //              Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        //         new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        // );

        // // Prevent the path from being flipped if the coordinates are already correct
        // path.preventFlipping = true;
        // followPathHolonomic = new FollowPathHolonomic(path, swerveSubsystem::getPose, swerveSubsystem::getFieldVelocity,
        //      swerveSubsystem::setChassisSpeeds, Constants.Auton.DRIVE_CONTROLLER_CONFIG, () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         }, swerveSubsystem);

        followPathHolonomic = AutoBuilder.pathfindToPose(Constants.FeildConstants.autoAlightAMPPose, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED));
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        System.out.println(followPathHolonomic.isFinished());
        return followPathHolonomic.isFinished();
    }
}
