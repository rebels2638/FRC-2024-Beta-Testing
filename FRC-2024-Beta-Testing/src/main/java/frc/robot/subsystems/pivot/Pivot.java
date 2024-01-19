package frc.robot.subsystems.pivot;

import java.util.Map;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private static final double kRadPositionTolerance = Math.toRadians(8);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private boolean velocityControlmode;
    PIDController positionFeedBackController;
    ArmFeedforward positionFeedForwardController;

    PIDController velocityFeedBackController;
    ArmFeedforward velocityFeedForwardController;

    private ShuffleboardTab tab = Shuffleboard.getTab("Turning");
    ShuffleboardTab pidTAB = Shuffleboard.getTab("PID Pose");
    GenericEntry pPoseSlide;
    GenericEntry iPoseSlide;
    GenericEntry dPoseSlide;
    GenericEntry sPoseSlide;
    GenericEntry gPoseSlide;
    GenericEntry vPoseSlide;

    public Pivot(PivotIO io)  {
        pPoseSlide = pidTAB.add("P Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        iPoseSlide = pidTAB.add("I Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        dPoseSlide = pidTAB.add("D Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();

        sPoseSlide = pidTAB.add("S Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        gPoseSlide = pidTAB.add("G Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        vPoseSlide = pidTAB.add("V Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();

        this.io = io;
        if (true) {
            positionFeedBackController = new PIDController(3, 0, 0);
            positionFeedForwardController = new ArmFeedforward(0, 0, 0);
            positionFeedBackController.setTolerance(kRadPositionTolerance);

            velocityFeedBackController = new PIDController(0, 0, 0);
            velocityFeedForwardController = new ArmFeedforward(0, 0, 0);


            io.configureController(positionFeedForwardController, positionFeedBackController,
                velocityFeedForwardController, velocityFeedBackController);
        }
        
    }

    @Override
    public void periodic() {
        positionFeedBackController.setP(pPoseSlide.getDouble(0));
        positionFeedBackController.setI(iPoseSlide.getDouble(0));
        positionFeedBackController.setD(dPoseSlide.getDouble(0));
        System.out.println(positionFeedBackController.getP());

        positionFeedForwardController =
         new ArmFeedforward(sPoseSlide.getDouble(0), 
         vPoseSlide.getDouble(0), gPoseSlide.getDouble(0));
        

        io.configureController(positionFeedForwardController, positionFeedBackController,
            velocityFeedForwardController, velocityFeedBackController);

        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Pivot", inputs);
    }

    public void setDegAngle(double angle) {
        Logger.getInstance().recordOutput("Pivot/desiredDegAngle", angle);
        io.setPosition(Math.toRadians(angle), inputs.positionRad);
        return;
    }

    public void setVelocityControlMode(boolean b){  
        velocityControlmode = b;
    };

    public void setVelocitySetPoint(double setPoint){
        io.setVelocity(setPoint, inputs.velocityRadSec);
        return;
    }
    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getDegAngle() {
        return inputs.positionDeg;
    }

    public double getRadAngle() {
        return inputs.positionRad;
    }

    public void zeroAngle() {
        io.zeroAngle();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint(velocityControlmode);
    }
}
