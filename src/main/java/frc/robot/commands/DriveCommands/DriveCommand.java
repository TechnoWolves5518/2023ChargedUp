// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants. DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final DoubleSupplier joystickX, joystickY, rotation;
  private final BooleanSupplier isFieldRelative;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, 
                      DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier rotation,
                      BooleanSupplier isFieldRelative) {
    this.driveSubsystem = driveSubsystem;
    this.joystickX = joystickX;
    this.joystickY = joystickY;
    this.rotation = rotation;
    this.isFieldRelative = isFieldRelative;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds;
    if (isFieldRelative.getAsBoolean()) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        joystickX.getAsDouble() * DriveConstants.maxVelocity, 
        joystickY.getAsDouble() * DriveConstants.maxVelocity,
        rotation.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond,
        driveSubsystem.getRotation2d()
      );
    } else {
      speeds = new ChassisSpeeds(
        joystickX.getAsDouble() * DriveConstants.maxVelocity,
        joystickY.getAsDouble() * DriveConstants.maxVelocity,
        rotation.getAsDouble() * DriveConstants.maxAngularSpeedRadiansPerSecond
      );
    }

    SwerveModuleState[] desiredStates = DriveConstants.driveKinematics.toSwerveModuleStates(speeds);
    driveSubsystem.setModuleStates(desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
