// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.aprilTagVision.AprilTagVision;

// public class ShooterWindup extends Command {
    
//     private final Shooter shooterSubsystem;
//     private final SwerveSubsystem swerveSubsystem;
//     private final AprilTagVision visionSubsystem;

//     Shoot(Shooter shooterSubsystem, SwerveSubsystem swerveSubsystem, AprilTagVision visionSubsystem){
//         this.shooterSubsystem = shootersubsystem;
//         this.swerveSubsystem = swerveSubsystem;
//         this.visionSubsystem = visionSubsystem;

//         addRequirements(shooterSubsystem);
//         addRequirements(swerveSubsystem);
//         addRequirements(visionSubsystem);
//     }

//     @Override
//     public void execute(){
//         ParallelCommandGroup group = new ParallelCommandGroup(
//             new AlignWithTargetPoint(this.shooterSubsystem, this.visionSubsystem, this.swerveSubsystem),
//             new ShooterWindup(this.shooterSubsystem, this.shooterSubsystem.getSpinSpeed()) // TODO: figure out how to extract SpinSpeed from subsystem through the 1st command
//         );
//         group.schedule();
//     }

//     @Override
//     public boolean isFinished(){
//         return true;
//     }
    
// }