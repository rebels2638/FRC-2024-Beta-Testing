package frc.robot.subsystems.poseLimelight;

public class PoseLimelightIOSim implements PoseLimelightIO{
    public void updateInputs(PoseLimelightIOInputs inputs) {
        inputs.botpose_wpiblue = new double[]{0, 0, 0, 0, 0, 0, 0};
        inputs.botpose_wpired = new double[]{0, 0, 0, 0, 0, 0, 0};
        inputs.cl = 0;
        inputs.tv = 0; 
    }

}