package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.limelight.PoseLimelight;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Utils.RebelUtil;

public class AlignWithTargetPoint extends Command {
    private final Shooter shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PoseLimelight llsubsystem;
    private final Pose3d shooterPose;

    public AlignWithTargetPoint(Shooter shooterSubsystem, SwerveSubsystem swerveSubsystem, PoseLimelight llsubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.llsubsystem = llsubsystem;
        this.shooterPose = new Pose3d(0,0,0,new Rotation3d(0,0,0)); // first three zeroes are real measurments, no euler angles (rot. at tip)
        addRequirements(shooterSubsystem, swerveSubsystem, llsubsystem);
    }

    @Override
    public void execute() {
        Pose2d initialPose = this.swerveSubsystem.getPose(); // assume that everything in x,y,z??
        Pose3d targetPointPose = this.llsubsystem.getValidShotPoint().relativeTo(new Pose3d(initialPose));

        Pose2d computedPose = RebelUtil.calculateAlignedPose(llsubsystem, initialPose, this.shooterPose, targetPointPose);
        RebelUtil.driveRobotToPose(computedPose);
        calcFlywheelSpeed(new Pose3d(computedPose), targetPointPose);

    }

    private static void calcFlywheelSpeed(Pose3d initial, Pose3d desired) {
        double wheelSpeed = 0.0;
        // actually implement physics for linear freefall to calc v_angular
        RebelUtil.flywheelSpeed = wheelSpeed;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}