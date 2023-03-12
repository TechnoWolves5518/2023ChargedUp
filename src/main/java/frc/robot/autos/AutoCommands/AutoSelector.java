package frc.robot.autos.AutoCommands;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDrive;
import frc.robot.autos.AutoDriveBase.AutoBalance;
import frc.robot.autos.AutoDriveBase.AutoDriveBack;
import frc.robot.autos.AutoDriveBase.AutoDriveForward;
import frc.robot.autos.AutoDriveBase.AutoLock;
import frc.robot.subsystems.Swerve;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  //define autonomous routines
  PathPlannerTrajectory ExamplePath = PathPlanner.loadPath("MConePickup", new PathConstraints(4, 3));
  PathPlannerTrajectory oneToLevel = PathPlanner.loadPath("OneToLevel", new PathConstraints(4, 3));
  PathPlannerTrajectory northAutoBalance = PathPlanner.loadPath("NorthAutoBalance", new PathConstraints(4, 2));
  PathPlannerTrajectory testPath = PathPlanner.loadPath("TestPath", new PathConstraints(4, 1));
  PathPlannerTrajectory southAutoBalance = PathPlanner.loadPath("SouthAutoBalance", new PathConstraints(4, 2));
  PathPlannerTrajectory northAutoBail = PathPlanner.loadPath("NorthAutoBail", new PathConstraints(4, 2));
  PathPlannerTrajectory southAutoBail = PathPlanner.loadPath("SouthAutoBail", new PathConstraints(4, 2));
  
  public AutoSelector(Swerve drivebase) {
    //I'm not gonna try and figure out something better for the time being
    chooser.setDefaultOption("NorthAutoLevel", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(northAutoBalance.getInitialHolonomicPose());
      }), 
      new PPSwerveControllerCommand(
        northAutoBalance,
         drivebase::getPose, // Pose supplier
       SwerveDrive.swerveKinematics, // SwerveDriveKinematics
         new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
         new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
         new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
         drivebase::setModuleStates, // Module states consumer
         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
         drivebase // Requires this drive subsystem
     ), new AutoDriveBack(drivebase), new AutoBalance(drivebase), new AutoLock(drivebase)
      ));

      chooser.addOption("SouthAutoLevel", new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(southAutoBalance.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          southAutoBalance,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       ), new AutoDriveBack(drivebase), new AutoBalance(drivebase), new AutoLock(drivebase)
        ));
      
      chooser.addOption("NorthAutoBail", new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(northAutoBail.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          northAutoBail,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
      ));

      chooser.addOption("SouthAutoBail", new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drivebase.resetOdometry(southAutoBail.getInitialHolonomicPose());
        }), 
        new PPSwerveControllerCommand(
          southAutoBail,
           drivebase::getPose, // Pose supplier
         SwerveDrive.swerveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           drivebase::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           drivebase // Requires this drive subsystem
       )
      ));

      chooser.addOption("Contingency", new SequentialCommandGroup(new AutoDriveForward(drivebase), 
      new AutoBalance(drivebase), 
      new AutoLock(drivebase)));
    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}