package frc.robot.subsystems.poseLimelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseLimelight extends SubsystemBase{
    private final PoseLimelightIOInputsAutoLogged inputs = new PoseLimelightIOInputsAutoLogged();
    private PoseLimelightIO io;
    public PoseLimelight(PoseLimelightIO poseLimelightIOReal) {
        this.io = poseLimelightIOReal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("PoseLimelight", inputs);

        // Logger.recordOutput("PoseLimelight/estRobotPose", getEstimatedRobotPose());
    }

    public boolean hasValidTargets() {
        return inputs.tv == 1;
    }

    public double getTargetArea() {
        return inputs.ta;
    }

    public double getAmbiguity() {
        return inputs.ambiguity;
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
}