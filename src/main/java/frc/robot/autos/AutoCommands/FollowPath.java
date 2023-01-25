// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FollowPath extends CommandBase {
  private Swerve s_Swerve;
  private String filePath;
  private boolean zeroInitialPose;

  PPSwerveControllerCommand followTrajectoryPathPlannerCommand;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
