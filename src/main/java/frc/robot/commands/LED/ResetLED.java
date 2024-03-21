package frc.robot.commands.LED;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ResetLED {

    public ResetLED() {
    if ((Shooter.getInstance().inShooter() || Intake.getInstance().inIntake())) {
        LEDSubsystem.getInstance().setColor(0.77); // green
    }

    else {
        LEDSubsystem.getInstance().setColor(0.61); // red
      }
    }
}
