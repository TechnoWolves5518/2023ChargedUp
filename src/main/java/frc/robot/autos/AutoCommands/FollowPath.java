// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class FollowPath extends CommandBase {
  private Swerve s_Swerve;
  private String filePath;
  private boolean zeroInitialPose;

  private PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
  private boolean done = false;
  /** Creates a new FollowPath. */
  public FollowPath(Swerve s_Swerve, String filePath, boolean zeroInitialPose) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.filePath = filePath;
    this.zeroInitialPose = zeroInitialPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //creates the trajectory
    PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(filePath, null);

    //reset the pose of the bot if true
    if (zeroInitialPose) {
      s_Swerve.resetOdometry(trajectoryToFollow.getInitialPose());
    }
    //PID controllers
    PIDController xController = new PIDController(PathPlannerConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(PathPlannerConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      PathPlannerConstants.kPThetaController, 0, 0, PathPlannerConstants.kThetaControllerConstraints);
    
      //create the PPswervecontroller command, allows for holonomic control
      followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
        trajectoryToFollow,
        s_Swerve.getPose(),
        SwerveDrive.swerveKinematics,
        xController,
        yController,
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve
      );

      followTrajectoryPathPlannerCommand.schedule();

  }

    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = followTrajectoryPathPlannerCommand.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
