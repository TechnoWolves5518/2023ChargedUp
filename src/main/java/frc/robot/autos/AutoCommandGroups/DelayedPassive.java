// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommandGroups;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveDrive;
import frc.robot.autos.AutoCommands.DelayDrive;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DelayedPassive extends SequentialCommandGroup {
  /** Creates a new DelayedPassive. */
  public DelayedPassive(Swerve drivebase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory toCone = PathPlanner.loadPath("ToCone", new PathConstraints(4, 2));
    addCommands(new DelayDrive(), new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      drivebase.resetOdometry(toCone.getInitialHolonomicPose());
    }),
    new PPSwerveControllerCommand(
      toCone,
       drivebase::getPose, // Pose supplier
     SwerveDrive.swerveKinematics, // SwerveDriveKinematics
       new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
       new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
       new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
       drivebase::setModuleStates, // Module states consumer
       false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
       drivebase // Requires this drive subsystem
   ));
  }
}
