package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignTrap extends Command {
    private Command followPathHolonomic;
    private SwerveSubsystem swerveSubsystem;
    
    public AutoAlignTrap(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d currentPose = swerveSubsystem.getPose();
        Pose2d bestPose2d = Constants.FieldConstants.autoAlignTrapPose[0];
        for (int i = 1; i < 3; i++) {
            double dist = calculateDistance(currentPose, Constants.FieldConstants.autoAlignTrapPose[i]);
            if (dist < calculateDistance(currentPose, bestPose2d)) {
                bestPose2d = Constants.FieldConstants.autoAlignTrapPose[i];
            }
        }
        
        followPathHolonomic = AutoBuilder.pathfindToPose(bestPose2d, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED));

        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        System.out.println("AutoAlignTrap");
        System.out.println(followPathHolonomic.isFinished());
        return followPathHolonomic.isFinished();
    }

    private double calculateDistance(Pose2d p1, Pose2d p2) {
        return p1.getTranslation().getDistance(p2.getTranslation());
    }
}
