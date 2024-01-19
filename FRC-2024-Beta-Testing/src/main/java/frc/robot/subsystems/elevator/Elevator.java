package frc.robot.subsystems.elevator;

import java.util.Map;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    private static final double kHightMetersPositionTolerance = .03;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    PIDController positionFeedBackController;
    ElevatorFeedforward positionFeedForwardController;

    PIDController velocityFeedBackController;
    ElevatorFeedforward velocityFeedForwardController;

    private ShuffleboardTab tab = Shuffleboard.getTab("Turning");
    ShuffleboardTab pidTAB = Shuffleboard.getTab("PID Pose");
    GenericEntry pPoseSlide;
    GenericEntry iPoseSlide;
    GenericEntry dPoseSlide;
    GenericEntry sPoseSlide;
    GenericEntry gPoseSlide;
    GenericEntry vPoseSlide;

    public Elevator(ElevatorIO io)  {
        pPoseSlide = pidTAB.add("P Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        iPoseSlide = pidTAB.add("I Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        dPoseSlide = pidTAB.add("D Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();

        sPoseSlide = pidTAB.add("S Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        gPoseSlide = pidTAB.add("G Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();
        vPoseSlide = pidTAB.add("V Value", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -8, "max", 8)).getEntry();

        this.io = io;
        if (true) {
            positionFeedBackController = new PIDController(3, 0, 0);
            positionFeedForwardController = new ElevatorFeedforward(0, 0, 0);
            positionFeedBackController.setTolerance(kHightMetersPositionTolerance);

            velocityFeedBackController = new PIDController(0, 0, 0);
            velocityFeedForwardController = new ElevatorFeedforward(0, 0, 0);


            io.configureController(positionFeedForwardController, positionFeedBackController);
        }
        
    }

    @Override
    public void periodic() {
        positionFeedBackController.setP(pPoseSlide.getDouble(0));
        positionFeedBackController.setI(iPoseSlide.getDouble(0));
        positionFeedBackController.setD(dPoseSlide.getDouble(0));
        System.out.println(positionFeedBackController.getP());

        positionFeedForwardController =
         new ElevatorFeedforward(sPoseSlide.getDouble(0), 
         vPoseSlide.getDouble(0), gPoseSlide.getDouble(0));
        

        io.configureController(positionFeedForwardController, positionFeedBackController);

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setHightMeters(double hightMeters) {
        io.setHightMeters(hightMeters, inputs.hightMeters);
        return;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
        return;
    }

    public double getHightMeters() {
        return inputs.hightMeters;
    }

    public void zeroHight() {
        io.zeroHight();
    }
    public boolean reachedSetpoint() {
        return io.reachedSetpoint();
    }
}
