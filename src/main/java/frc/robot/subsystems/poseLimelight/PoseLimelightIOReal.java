package frc.robot.subsystems.poseLimelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PoseLimelightIOReal implements PoseLimelightIO {
    private NetworkTable llTable;
    public PoseLimelightIOReal() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");

    }
    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.botpose_wpired = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        inputs.cl = llTable.getEntry("cl").getDouble(0);
        inputs.tv = llTable.getEntry("tv").getDouble(0);
        inputs.ta = llTable.getEntry("ta").getDouble(0);
        // [id,txnc,tync,ta,distToCamera,distToRobot,ambiguity,id2.....]
        inputs.ambiguity = llTable.getEntry("rawfiducials").getDoubleArray(new double[7])[6];
    }

}