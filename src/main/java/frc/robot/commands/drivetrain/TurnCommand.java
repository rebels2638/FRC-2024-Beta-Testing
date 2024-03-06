package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;

import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class TurnCommand extends Command {

    private final SwerveSubsystem swerve;
    private Pose2d lastPose;
    private double yaw;

    public TurnCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        lastPose = this.swerve.getPose();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.lastPose = this.swerve.getPose();
    }

    @Override
    public void execute() {
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(0, 0,
                                                        lastPose.getRotation().rotateBy(new Rotation2d(Math.toRadians(90))));
        swerve.drive(new Translation2d(0,0), desiredSpeeds.omegaRadiansPerSecond, true, false, true);
                                                        //  end(true);\
        yaw = swerve.getYaw().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0), 0, true, false, true);
        // TODO Auto-generated method stub
    }

}