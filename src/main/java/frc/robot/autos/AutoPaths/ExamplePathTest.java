// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoPaths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class ExamplePathTest extends CommandBase {
  Swerve s_Swerve;
  PathPlannerTrajectory mConePickup;
  public ExamplePathTest(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
   addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectory mConePickup = PathPlanner.loadPath("MConePickup", new PathConstraints(4, 3));
    new PPSwerveControllerCommand(
         mConePickup,
          s_Swerve::getPose, // Pose supplier
        SwerveDrive.swerveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          s_Swerve::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          s_Swerve // Requires this drive subsystem
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
