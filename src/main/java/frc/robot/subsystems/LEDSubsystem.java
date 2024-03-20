// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDSubsystem extends SubsystemBase {
public Spark colorBlinkin = new Spark(8);
// public DigitalOutput colorBlinkin = new DigitalOutput(6);

public static LEDSubsystem instance = null;
// public Spark brightness = new Spark(5);
  /** Creates a new ExampleSubsystem. */
  public LEDSubsystem() {
    // colorBlinkin.set(0.87);
  }

  /**
   * Sets the voltage(s) for each 
   * @param a Voltage for color1
   * @return
   */
  public void setColor(double a) {
    colorBlinkin.set(a);
    // colorBlinkin.pulse(a);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public static LEDSubsystem setInstance(LEDSubsystem c){
    if(instance == null){
        instance = c;
    }
    return instance;
  }
  /**
   * INSTANCE INSTANCE INSTANCE INSTANCE
   * @return
   */
  public static  LEDSubsystem getInstance(){
    if(instance == null){
        instance = new LEDSubsystem();
    }
    return instance;
  }
}
