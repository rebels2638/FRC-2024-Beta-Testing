package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.limelight.PoseLimelight;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;;

// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// acts more like a helper class rather than a subsystem or command.
public class AutoRunner {


    private final SendableChooser<String> pathChooser = new SendableChooser<String>();
    private String pathChosen = "taxi";
    private static final HashMap<String, String> PATH_CHOSEN_TO_NAME_HASH_MAP = new HashMap<>();
    private static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
    // private static PathPlannerAuto pathPlanner = new PathPlannerAuto();
    // private static PPSwerveControllerCommand scc;
    // private static HolonomicDriveController hdc = new HolonomicDriveController(
    //   new PIDController(0.1, 0, 0), new PIDController(.1, 0, 0), new ProfiledPIDController(0.1, 0, 0,new Constraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION)));
    private static List<com.pathplanner.lib.path.PathPlannerTrajectory> pathList;
    private static PathPlannerPath path;
    private static PathfindThenFollowPathHolonomic pathCommand;


    static {
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidTop", "3PMidTop");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidMA", "3PMidMA");
    }
    
    public AutoRunner (SwerveSubsystem swerveSubsystem) {
        //NamedCommands.registerCommand("MoveElevatorAMP", new MoveElevatorAMP(elevatorSubsystem));

        PATH_CHOSEN_TO_NAME_HASH_MAP.forEach((pathName, pathFile) -> pathChooser.addOption(pathName, pathFile));

        Shuffleboard.getTab("Auto").add("Path Chooser", pathChooser);
        // Shuffleboard.getTab("Auto").add("Update Selected Command Output", 
        //     new InstantCommand( () -> loadPath()));
        
        AutoBuilder.configureHolonomic(
                // visionSubsystem::getBotPose2d,
                swerveSubsystem::getPose, // Robot pose supplier
                swerveSubsystem::resetOdometryAuto, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveSubsystem::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerveSubsystem::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        Constants.Auton.TRANSLATION_PID_CONFIG, // Translation PID constants
                        Constants.Auton.ANGLE_PID_CONFIG, // Rotation PID constants
                        Constants.Auton.MAX_SPEED, // Max module speed, in m/s
                        0.5384061785684372, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return true;
                },
                swerveSubsystem // Reference to this subsystem to set requirements
        ); 
    }

    private void loadPath() {
        pathChosen = pathChooser.getSelected();
        Shuffleboard.getTab("Auto").add("Selected Path", pathChosen);
    }

        
    public Command getAutonomousCommand() { 
        //TODO:!! HERE CHANGE THIS 
        //4PMidMASScoreTurn
        //3PAmpAM << Amp Side 3 piece      
        //2PAmp << 2 piece Amp'
        //TheFuckYouAuto << messes up midline
        //TheFuckMeAuto << messes up midline
        //AmpMidlineDisrupt << Starts form AMp
        //3.5PMidMSFar1
        //3PAmpA1
        //3PMidMAScoreTurn
        //TT << turnTest
        //4PAmpA12
        return new PathPlannerAuto("AmpMidlineDisrupt");
    }
} 