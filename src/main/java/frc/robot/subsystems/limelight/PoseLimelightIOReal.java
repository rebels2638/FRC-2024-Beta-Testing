package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utils.LimelightHelpers;

public class PoseLimelightIOReal implements PoseLimelightIO {
    private NetworkTable table;
    private int pipelineNumber = 0; // configure in limelight application

    public PoseLimelightIOReal() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(pipelineNumber);
    }

    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.botpose_wpired = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        inputs.cl = table.getEntry("cl").getDouble(0);

        inputs.tv = table.getEntry("tv").getDouble(0);
        inputs.tx = table.getEntry("tx").getDouble(0);
        inputs.ty = table.getEntry("ty").getDouble(0);
        inputs.ta = table.getEntry("ta").getDouble(0);
    }

    public void setID(int number) {
        for (int i = 0; getID() != number) && i < 6; i++)
            Results.targets_Fiducials.getNext();
    }

    public double getID() {
        return Results.getFiducialID("limelight");
    }

    public double getV() { return inputs.tv; }

    public double getX() { return inputs.tx; }
    
    public double getY() { return inputs.ty; }

    public double getA() { return inputs.ta; }

}
