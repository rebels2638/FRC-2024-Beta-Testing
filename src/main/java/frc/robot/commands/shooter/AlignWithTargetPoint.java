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
    private final SwerveSubsystem swerveSubsystem;
    private final PoseLimelight llsubsystem;
    private final Pose3d shooterPose;

    public AlignWithTargetPoint(SwerveSubsystem swerveSubsystem, PoseLimelight llsubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.llsubsystem = llsubsystem;
        this.shooterPose = new Pose3d(-0.301828,0,0.577596,new Rotation3d(0,0,0)); // first three zeroes are real measurments, no euler angles (rot. at tip)
        addRequirements(swerveSubsystem, llsubsystem); // F_g = mg moment
    }

    @Override
    public void execute() {
        Pose2d currPose = this.swerveSubsystem.getPose(); // assume that everything in x,y,z??
        Pose2d initialPose = new Pose2d(currPose.getX()-(15/39.4), currPose.getY(), currPose.getRotation()); // adjusted for the 15 in offset :skull:
        Pose3d targetPointPose = this.llsubsystem.getValidShotPoint().relativeTo(new Pose3d(initialPose));

        Pose2d computedPose = RebelUtil.calculateAlignedPose(llsubsystem, initialPose, this.shooterPose, targetPointPose);
        RebelUtil.driveRobotToPose(computedPose);
        calcFlywheelSpeed(new Pose3d(computedPose), targetPointPose);
    }

    private static void calcFlywheelSpeed(Pose3d initial, Pose3d desired) {
        RebelUtil.flywheelSpeed = Math.sqrt((8/3)*9.8*Math.abs(initial.getZ()-desired.getZ()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}