package frc.robot.commands.autoAligment;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class MoveAndShootUtil {
    // private final SwerveSubsystem swerveSubsystem; 
    // private final Shooter shooterSubsystem; 
    
    private static final Translation3d SPEAKER_CENTER_TRANSLATION = Constants.FieldConstants.SPEAKER_CENTER_TRANSLATION;

    // public MoveAndShootUtil(SwerveSubsystem swerveSubsystem, Shooter shooterSubsystem) {
    //     this.swerveSubsystem = swerveSubsystem;
    //     this.shooterSubsystem = shooterSubsystem;
    // }

    public static Rotation2d getValidRobotRotation(ChassisSpeeds robotFeildRelitiveChassisSpeeds, Translation2d robotTranslation, Translation3d targetPoint) {
        Rotation2d calculatedRot = new Rotation2d(Math.atan2(targetPoint.getY() - robotTranslation.getY(), targetPoint.getX() - robotTranslation.getX()));
        Logger.recordOutput("SpeakerAligment/calculatedRobotRotationRad", calculatedRot);

        return calculatedRot;
    }

    public static Rotation2d getShooterAngel(ChassisSpeeds robotFeildRelitiveChassisSpeeds, Translation2d robotTranslation, Translation3d targetPoint) {
        // move everything down so shooter is at the "origin"
        targetPoint = new Translation3d(targetPoint.getX(), targetPoint.getY(), targetPoint.getZ() - Constants.Drivebase.SHOOTER_POSE.getZ());

        // make a plane
        double x = Math.sqrt(Math.pow(robotTranslation.getX() - targetPoint.getX(), 2) + 
                Math.pow(robotTranslation.getY() - targetPoint.getY(), 2));

        double y = targetPoint.getZ();
        Logger.recordOutput("SpeakerAligment/x", x);
        Logger.recordOutput("SpeakerAligment/y", y);

        
        double g = 9.8;

        double radical = Math.pow(Shooter.noteExitVelocityMetersPerSec, 4) - 
        g * (g * Math.pow(x, 2) + 2 * y * Math.pow(Shooter.noteExitVelocityMetersPerSec, 2));

        // no valid shots givin the velocity
        if (radical < 0) {
            return null;
        }

        double a1 = Math.atan(
            (Math.pow(Shooter.noteExitVelocityMetersPerSec, 2) +
            Math.sqrt(radical))
            / g * x);
        
        double a2 = Math.atan(
            (Math.pow(Shooter.noteExitVelocityMetersPerSec, 2) -
            Math.sqrt(radical))
            / g * x);

        double bestAngle = a2;
        if (a1 > a2) {
            bestAngle = a1;
        }

        return new Rotation2d(bestAngle);
        
    }

    public static Translation3d getTargetPoint(ChassisSpeeds robotFeildRelitiveChassisSpeeds, Translation2d robotTranslation) {
        Translation3d shooterTranslation = addTranslations(robotTranslation, Constants.Drivebase.SHOOTER_POSE.getTranslation());

        double noteTravelDistMeters = Math.sqrt(Math.pow(shooterTranslation.getX() - SPEAKER_CENTER_TRANSLATION.getX(), 2) +
                                                Math.pow(shooterTranslation.getY() - SPEAKER_CENTER_TRANSLATION.getY(), 2) +
                                                Math.pow(shooterTranslation.getZ() - SPEAKER_CENTER_TRANSLATION.getZ(), 2));

        // Logger.recordOutput("SpeakerAligment/noteTravelDistMeters", noteTravelDistMeters);

        double x = Math.sqrt(Math.pow(robotTranslation.getX() - SPEAKER_CENTER_TRANSLATION.getX(), 2) + 
                Math.pow(robotTranslation.getY() - SPEAKER_CENTER_TRANSLATION.getY(), 2));
        
        Rotation2d estAngle = getShooterAngel(robotFeildRelitiveChassisSpeeds, robotTranslation, SPEAKER_CENTER_TRANSLATION);
        
        double estAirTimeSec;
        if (estAngle == null) {
            estAirTimeSec = noteTravelDistMeters / Shooter.noteExitVelocityMetersPerSec;
            Logger.recordOutput("SpeakerAligment/estAirTimeSec", estAirTimeSec);

        }
        else {
            estAirTimeSec = x / Shooter.noteExitVelocityMetersPerSec * 
            getShooterAngel(robotFeildRelitiveChassisSpeeds, robotTranslation, SPEAKER_CENTER_TRANSLATION).getCos();
            Logger.recordOutput("SpeakerAligment/estAirTimeShooterAngle", getShooterAngel(robotFeildRelitiveChassisSpeeds, robotTranslation, SPEAKER_CENTER_TRANSLATION).getDegrees());
            Logger.recordOutput("SpeakerAligment/estAirTimeSecParabola", estAirTimeSec);

        }


        double xComp, yComp;
        if (robotFeildRelitiveChassisSpeeds.vxMetersPerSecond == 0.0) {
            xComp = 0;
        }
        else {
            xComp = estAirTimeSec * robotFeildRelitiveChassisSpeeds.vxMetersPerSecond;
        }

        if (robotFeildRelitiveChassisSpeeds.vyMetersPerSecond == 0.0) {
            yComp = 0;
        }
        else {
            yComp = estAirTimeSec * robotFeildRelitiveChassisSpeeds.vyMetersPerSecond;
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
