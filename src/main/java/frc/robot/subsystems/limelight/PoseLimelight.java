package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.PoseLimelightIO.PoseLimelightIOInputs;

public class PoseLimelight extends SubsystemBase{
    private final PoseLimelightIOInputs inputs = new PoseLimelightIOInputs();
    private PoseLimelightIO io;

    // apriltag constants,, fix all 
    private final Pose3d defaultShotPoint = new Pose3d(-1.50, 196.17, 2.1, new Rotation3d(0, 0, 0));
    private final double max_tx = 0;
    private final double max_ta = 0;
    private final double maxTranslationY = 0.05;
    private final double maxTranslationZ = 0.2;

    public PoseLimelight(PoseLimelightIO poseLimelightIOReal) {
        this.io = poseLimelightIOReal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("PoseLimelight", inputs);
    }

    public boolean hasValidTargets() {
        return inputs.tv == 1;
    }
    
    public Pose2d getEstimatedRobotPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(inputs.botpose_wpired[0], inputs.botpose_wpired[1]), new Rotation2d(inputs.botpose_wpired[5]));
        }
        
        return new Pose2d(new Translation2d(inputs.botpose_wpiblue[0], inputs.botpose_wpiblue[1]), new Rotation2d(inputs.botpose_wpiblue[5]));
    }

    public double getTimestampSeconds() {
        return Timer.getFPGATimestamp() - (inputs.botpose_wpiblue[6]/1000.0);
    }

    // apriltag methods
    public Pose3d getValidShotPoint() {
        double x = defaultShotPoint.getX();

        double proportionX = Math.abs(io.getXLimelight() / max_tx);
        double y = defaultShotPoint.getY() + (proportionX * maxTranslationY);

        double proportionA = io.getALimelight() / max_ta;
        double z = defaultShotPoint.getZ() + (proportionA * maxTranslationZ);

        return new Pose3d(x, y, z, new Rotation3d(0, 0, 0));
    }

}
