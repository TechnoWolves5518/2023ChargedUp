// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrive;
import frc.robot.Constants.SwerveDrive.CameraConstants;
import frc.robot.subsystems.Swerve;

public class AimTarget extends CommandBase {
  
  Swerve s_Swerve;
  double rotationSpeed;
  double range;
  double forwardSpeed;

  public AimTarget() {
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  //based on the photonlib example, unclear if it actually works but this could help to aim at the apriltag, granted pose estimation would be better
  @Override
  public void execute() {
    var result = CameraConstants.camera.getLatestResult();
    if (result.hasTargets()) {
      range = PhotonUtils.calculateDistanceToTargetMeters(CameraConstants.cameraHeightMeters, CameraConstants.goalDistanceMeters, CameraConstants.cameraAngleRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
      forwardSpeed = -CameraConstants.driveController.calculate(range, CameraConstants.goalDistanceMeters);
      rotationSpeed = -CameraConstants.driveController.calculate(result.getBestTarget().getYaw(), 0);
      s_Swerve.drive(new Translation2d(forwardSpeed,0), rotationSpeed, true, true);
    } else {
      rotationSpeed = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/