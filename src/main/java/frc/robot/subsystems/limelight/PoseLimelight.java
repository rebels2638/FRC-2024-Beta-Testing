package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseLimelight extends SubsystemBase{
    private final PoseLimelightIOInputsAutoLogged inputs = new PoseLimelightIOInputsAutoLogged();
    private PoseLimelightIO io;
    // private final Pose3d[] bounding_box = new Pose3d[2];

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public PoseLimelight(PoseLimelightIO poseLimelightIOReal) {
        this.io = poseLimelightIOReal;
        
        // bounding_box[0] = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        // bounding_box[1] = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
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
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Pose2d(new Translation2d(inputs.botpose_wpired[0], inputs.botpose_wpired[1]), new Rotation2d(inputs.botpose_wpired[5]));
        }
        
        return new Pose2d(new Translation2d(inputs.botpose_wpiblue[0], inputs.botpose_wpiblue[1]), new Rotation2d(inputs.botpose_wpiblue[5]));
    }
/*
    public Pose3d getValidShotPoint() {
        Pose2d pose = getEstimatedRobotPose();
        
        Pose3d[] ro_bounding_box = new Pose3d[2];
        ro_bounding_box[0] = new Pose3d(pose.getX()-bounding_box[0].getX(), pose.getY()-bounding_box[0].getY(), bounding_box[0].getZ(), new Rotation3d(0, 0, 0));
        ro_bounding_box[1] = new Pose3d(pose.getX()-bounding_box[1].getX(), pose.getY()-bounding_box[1].getY(), bounding_box[1].getZ(), new Rotation3d(0, 0, 0));   

        Rotation3d[] euler_bounding_box = new Rotation3d[2];
        euler_bounding_box[0] = new Rotation3d(ro_bounding_box[0].getRotation().getX(), ro_bounding_box[0].getRotation().getY(), ro_bounding_box[0].getRotation().getZ());
        euler_bounding_box[1] = new Rotation3d(ro_bounding_box[1].getRotation().getX(), ro_bounding_box[1].getRotation().getY(), ro_bounding_box[1].getRotation().getZ());
              
        double x = ;

        Rotation3d apriltag = getAprilTagPose();
        double roll = apriltag.getX();
        double z = Math.min(ro_bounding_box[0].getZ(), ro_bounding_box[1].getZ()); 
        
        return new Pose3d(x, y, z, new Rotation3d(0, 0, 0));
    }

    public Rotation3d getAprilTagPose() {
        if (table.getEntry("tv").getDouble(0.0) < 1) {
            return null;
        }

        NetworkTableEntry tx = table.getEntry("tx");
        double x = -tx.getDouble(0.0);
    
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);

        return new Rotation3d(x, y, 0);
    }
*/
    public double getTimestampSeconds() {
        return Timer.getFPGATimestamp() - (inputs.botpose_wpiblue[6]/1000.0);
    }
}
