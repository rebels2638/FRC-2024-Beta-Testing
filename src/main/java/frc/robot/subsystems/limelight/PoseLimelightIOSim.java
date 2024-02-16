package frc.robot.subsystems.limelight;

public class PoseLimelightIOSim implements PoseLimelightIO{
    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = new double[] {0, 0, 0, 0, 0, 0, 0};
        inputs.botpose_wpired = new double[] {0, 0, 0, 0, 0, 0, 0};
        inputs.cl = 0;
        inputs.tv = 0; 
    }

    // public void setID();

    @Override
    public double getID() {return 0.0;}

    @Override
    public double getV() {return 0.0;}

    @Override
    public double getX() {return 0.0;}
    
    @Override
    public double getY() {return 0.0;}

    @Override
    public double getA() {return 0.0;}

}
