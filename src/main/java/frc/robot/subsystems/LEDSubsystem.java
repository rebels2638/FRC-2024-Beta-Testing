// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

// import com.revrobotics.;

public class LEDSubsystem extends SubsystemBase {
public Spark colorBlinkin = new Spark(8); //8
// public DigitalOutput colorBlinkin = new DigitalOutput(6);

public static LEDSubsystem instance = null;
double value;
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
    value = a;
    colorBlinkin.set(a);
    // colorBlinkin.pulse(a);
  }

  @Override
  public void periodic() {
    // colorBlinkin.set(0.69);
    // colorBlinkin.set(value);
    double color = 0.0;
    if (DriverStation.isAutonomous()) {colorBlinkin.set(0.87);} // blue

    else {

      if ((Shooter.getInstance().inShooter() || Intake.getInstance().inIntake())) {
        color = 0.77; // green
      }

      // if (Shooter.getInstance().getDesiredVelocity() > 6.2 && Shooter.getInstance().getVelocityRadSec() <= 60) {
      //   color = 0.15; // strobe
      // }

      if (Shooter.getInstance().getDesiredVelocity() == 60 && Shooter.getInstance().getVelocityRadSec() < 60) {
        color = 0.15; // strobe current color?
      }

      if (Shooter.getInstance().getDesiredVelocity() == 48 && Shooter.getInstance().getVelocityRadSec() < 48) {
        color = -0.05; // strobe white
      } 

      if (Shooter.getInstance().getDesiredVelocity() == -24 && Shooter.getInstance().getVelocityRadSec() > -24) {
        color = -0.07; // strobe gold
      }

      else {
        color = 0.61; // red
      }
      colorBlinkin.set(color);

    }

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
