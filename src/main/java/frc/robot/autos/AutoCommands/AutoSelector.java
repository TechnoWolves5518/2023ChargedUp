package frc.robot.autos.AutoCommands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;



public class AutoSelector {
    
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  //define autonomous routines
  PathPlannerTrajectory ExamplePath = PathPlanner.loadPath("MConePickup", new PathConstraints(4, 3));

  public AutoSelector(Swerve drivebase) {
    chooser.setDefaultOption("ExamplePath", new SequentialCommandGroup(

    new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        drivebase.resetOdometry(ExamplePath.getInitialHolonomicPose());
      }),
      new PPSwerveControllerCommand(
         ExamplePath,
          drivebase::getPose, // Pose supplier
        SwerveDrive.swerveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivebase::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drivebase // Requires this drive subsystem
      ), new AutoBalance(drivebase)
      ));


    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}