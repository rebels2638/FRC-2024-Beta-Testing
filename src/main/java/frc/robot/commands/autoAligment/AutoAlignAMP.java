package frc.robot.commands.autoAligment;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignAMP extends Command {
    private Command followPathHolonomic;
    private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    
    public AutoAlignAMP() {
    }

    // Builds a follow path holonomic path using field constants.
    @Override
    public void initialize() {
        followPathHolonomic = AutoBuilder.pathfindToPose(Constants.FieldConstants.autoAlignAMPPose, new PathConstraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION, 
            Constants.Auton.MAX_ANGULAR_VELO_RPS, Constants.Auton.MAX_ANGULAR_ACCEL_RPS_SQUARED));
        CommandScheduler.getInstance().schedule(followPathHolonomic);
    }

    @Override   
    public boolean isFinished() {
        System.out.println(followPathHolonomic.isFinished());
        return followPathHolonomic.isFinished();
    }
}
