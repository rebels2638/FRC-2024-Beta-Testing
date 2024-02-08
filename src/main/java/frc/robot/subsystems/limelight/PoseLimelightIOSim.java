package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class PoseLimelightIOSim implements PoseLimelightIO{
    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = new double[]{0, 0, 0, 0, 0, 0, 0};
        inputs.botpose_wpired = new double[]{0, 0, 0, 0, 0, 0, 0};
        // inputs.botpose_wpired = new Pose3d(0,0,0,new Rotation3d(0,0,0));
        // inputs.botpose_wpiblue = new Pose3d(0,0,0,new Rotation3d(0,0,0));
        inputs.cl = 0;
        inputs.tv = 0; 
    }

}
