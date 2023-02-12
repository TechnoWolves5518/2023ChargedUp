// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class ChargeBalance extends CommandBase {
  
  Swerve s_Swerve;
  double elevationAngle;
  double errorThreshold;
  double yawError;

  public ChargeBalance(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    elevationAngle = s_Swerve.getElevationAngle();
    yawError = s_Swerve.getRawYaw();
    //System.out.println(elevationAngle);
    System.out.println(yawError);
    if (elevationAngle > AutoConstants.maxPlatformPositivePitch) {
      s_Swerve.drive(
        new Translation2d(SwerveDrive.balanceSpeedMod,0),
        0,
        true,
        true
      );
      } else if (elevationAngle < AutoConstants.maxPlatformNegativePitch) {
        s_Swerve.drive(new Translation2d(-SwerveDrive.balanceSpeedMod,0), 
        0, 
        true, 
        true);
      } else {
        s_Swerve.drive(new Translation2d(0,0), 
        0, 
        true, 
        true);
      }
    }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
