package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class MoveAndShootUtil {
    // private final SwerveSubsystem swerveSubsystem; 
    // private final Shooter shooterSubsystem; 
    
    private static final double noteExitVelocityMetersPerSec = 1.5;

    private static final Translation3d SPEAKER_CENTER_TRANSLATION = Constants.FieldConstants.SPEAKER_CENTER_TRANSLATION;

    // public MoveAndShootUtil(SwerveSubsystem swerveSubsystem, Shooter shooterSubsystem) {
    //     this.swerveSubsystem = swerveSubsystem;
    //     this.shooterSubsystem = shooterSubsystem;
    // }

    public static Rotation2d getValidRobotRotation(ChassisSpeeds robotFeildRelitiveChassisSpeeds, Translation2d robotTranslation) {
        Translation3d targetPoint = getTargetPoint(robotFeildRelitiveChassisSpeeds, addTranslations(robotTranslation, Constants.Drivebase.SHOOTER_POSE.getTranslation()));

        Rotation2d calculatedRot = new Rotation2d(Math.atan2(targetPoint.getY() - robotTranslation.getY(), targetPoint.getX() - robotTranslation.getX()));
        Logger.recordOutput("SpeakerAligment/calculatedRobotRotationRad", calculatedRot);

        return calculatedRot;
    }

    private static Translation3d getTargetPoint(ChassisSpeeds robotFeildRelitiveChassisSpeeds, Translation3d shooterTranslation) {
        double noteTravelDistMeters = Math.sqrt(Math.pow(shooterTranslation.getX() - SPEAKER_CENTER_TRANSLATION.getX(), 2) +
                                                Math.pow(shooterTranslation.getY() - SPEAKER_CENTER_TRANSLATION.getY(), 2) +
                                                Math.pow(shooterTranslation.getZ() - SPEAKER_CENTER_TRANSLATION.getZ(), 2));

        Logger.recordOutput("SpeakerAligment/noteTravelDistMeters", noteTravelDistMeters);
        Logger.recordOutput("SpeakerAligment/shooterTranslation", shooterTranslation);

        // TODO: CHECK THIS! ALSO, this is NOT a paroblic path!
        double estAirTimeSec = noteTravelDistMeters / Shooter.noteExitVelocityMetersPerSec;
        Logger.recordOutput("SpeakerAligment/estAirTimeSec", estAirTimeSec);

        double xComp, yComp;
        if (robotFeildRelitiveChassisSpeeds.vxMetersPerSecond == 0.0) {
            xComp = 0;
        }
        else {
            xComp = estAirTimeSec / robotFeildRelitiveChassisSpeeds.vxMetersPerSecond;
        }

        if (robotFeildRelitiveChassisSpeeds.vyMetersPerSecond == 0.0) {
            yComp = 0;
        }
        else {
            yComp = estAirTimeSec / robotFeildRelitiveChassisSpeeds.vyMetersPerSecond;
        }

        Logger.recordOutput("SpeakerAligment/xComp", xComp);
        Logger.recordOutput("SpeakerAligment/yComp", yComp);
        Logger.recordOutput("SpeakerAligment/vxMetersPerSecond", robotFeildRelitiveChassisSpeeds.vxMetersPerSecond);

        Translation3d targetPoint = new Translation3d(SPEAKER_CENTER_TRANSLATION.getX() - xComp,
                                                     SPEAKER_CENTER_TRANSLATION.getY() - yComp,
                                                     SPEAKER_CENTER_TRANSLATION.getZ());

        // Translation3d targetPoint = new Translation3d(SPEAKER_CENTER_TRANSLATION.getX() - robotFeildRelitiveChassisSpeeds.vxMetersPerSecond / estAirTimeSec,
        //                                              SPEAKER_CENTER_TRANSLATION.getY() - robotFeildRelitiveChassisSpeeds.vyMetersPerSecond / estAirTimeSec,
        //                                              SPEAKER_CENTER_TRANSLATION.getZ());

        double[] loggingArr = {targetPoint.getX(), targetPoint.getY(), targetPoint.getZ()};

        Logger.recordOutput("SpeakerAligment/targetPoint", loggingArr);
        return targetPoint;
        
    }

    private static Translation3d addTranslations(Translation2d a, Translation3d b) {
        return new Translation3d(a.getX() + b.getX(), a.getY() + b.getY(), b.getZ());
    }

}
