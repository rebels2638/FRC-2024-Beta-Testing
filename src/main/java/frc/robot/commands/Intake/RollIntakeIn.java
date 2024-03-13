package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RollIntakeIn extends Command {
    private final Intake intakeSubsystem = Intake.getInstance();
    private final Pivot pivotSubsystem = Pivot.getInstance();
    
    public RollIntakeIn() {
    }

    @Override
    public void initialize() { 
        if (pivotSubsystem.getDegAngle() < 45) {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 10)); //Use radians directly.
        } else {
            intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 8)); //Use radians directly.
            // intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 8.6) - ); //Use radians directly.

            // double robotVeloY = SwerveSubsystem.getInstance().getRobotVelocity().vyMetersPerSecond;
            // double robotVeloX = SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond;

            // double robotVelo = Math.sqrt(Math.pow(robotVeloX, 2)  + Math.pow(robotVeloY, 2)) * 0.5;
            // // robotVelo = 
            // // intakeSubsystem.setVelocityRadSec(
            // //     (SwerveSubsystem.getInstance().getRobotVelocity().vyMetersPerSecond / Constants.Drivebase.MAX_TRANSLATIONAL_VELOCITY_METER_PER_SEC)
            // //     *Math.toRadians(360*5.6) + 360*3);
            // // double normalVeloRadSec = 12;
            // intakeSubsystem.setVelocityRadSec(Math.toRadians(360 * 8.6) - robotVelo); //Use radians directly.
            // intakeSubsystem.setVelocityRadSec(normalVeloRadSec / robotVelo );
        }
    }
    
    @Override
    public void end(boolean isInterrupted){
        // intakeSubsystem.setVelocityRadSec(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
