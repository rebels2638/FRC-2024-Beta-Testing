package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.PoseLimelightIO.PoseLimelightIOInputs;
import frc.robot.subsystems.pivot.Pivot;

import java.util.Optional;

public class PoseLimelight extends SubsystemBase{

    private final PoseLimelightIOInputsAutoLogged inputs = new PoseLimelightIOInputsAutoLogged();
    private static PoseLimelightIO io;
    public final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    // apriltag constants,, fix all 
    private final Pose3d defaultShotPoint;
    private final double max_delta_tx = 0; // in degrees
    private final double max_ta = 0; // in meters (i think)
    private final double maxTranslationY = 0.05;
    private final double maxTranslationZ = 0.2;

    private static PoseLimelight instance;

    public PoseLimelight(PoseLimelightIO poseLimelightIOReal) {
        this.io = poseLimelightIOReal;
        if (alliance.isPresent()) {
            defaultShotPoint = (alliance.get() == DriverStation.Alliance.Red)
                ? new Pose3d(16.579342, 5.547868, 2.1, new Rotation3d(0, 0, 0)
                ) : new Pose3d(-0.0381, 5.547868, 2.1, new Rotation3d(0, 0, 0));
        }
        else {
            defaultShotPoint = new Pose3d(16.579342, 5.547868, 2.1, new Rotation3d(0, 0, 0));
        }

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PoseLimelight", inputs);
    }

    public boolean hasValidTargets() {
        return inputs.tv == 1;
    }
    
    public Pose2d getEstimatedRobotPose() {
        
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

        // double proportionX = Math.abs(io.getX() / max_delta_tx);
        // double y = defaultShotPoint.getY() + (proportionX * maxTranslationY);

        double proportionA = io.getA() / max_ta;
        double z = defaultShotPoint.getZ() + (proportionA * maxTranslationZ);

        return new Pose3d(x, defaultShotPoint.getY(), z, new Rotation3d(0, 0, 0));
    }

    public Pose2d getDefaultAlignPoint() {
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red)
                ? new Pose2d(16.579342, 5.547868, new Rotation2d(180) // 1.34, 5.51
                ) : new Pose2d(-0.0381, 5.547868, new Rotation2d(180)); // 
        }
        return new Pose2d(16.579342, 5.547868, new Rotation2d(180));
    }

    public static PoseLimelight setInstance(PoseLimelight p) {
        PoseLimelight.instance = p;
        return p;
    }

    public static PoseLimelight getInstance(){
        if (PoseLimelight.instance == null){
            return new PoseLimelight(PoseLimelight.io);
        }
        return instance;
    }

}
