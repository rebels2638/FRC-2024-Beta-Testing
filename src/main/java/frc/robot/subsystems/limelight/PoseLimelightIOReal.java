package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseLimelightIOReal implements PoseLimelightIO {
    private NetworkTable llTable;
    public PoseLimelightIOReal() {
        llTable = NetworkTableInstance.getDefault().getTable("PoseLimelight");

    }
    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.botpose_wpired = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        inputs.cl = llTable.getEntry("cl").getDouble(0);
        inputs.tv = llTable.getEntry("tv").getDouble(0);
    }

}
