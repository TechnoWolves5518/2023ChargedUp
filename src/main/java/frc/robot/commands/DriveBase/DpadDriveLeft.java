// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrive;
import frc.robot.subsystems.Swerve;

public class DpadDriveLeft extends CommandBase {
  Swerve s_Swerve;
  Joystick override;
  boolean stopCheck;
  boolean convertedStopCheck;
  int timer;
  public DpadDriveLeft(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    override = new Joystick(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = override.getRawButton(1);
    if (stopCheck == true) {
      convertedStopCheck = false;
    } else {
      convertedStopCheck = true;
    }
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer < 2) {
      timer++;
    } else {
    stopCheck = override.getRawButton(1);
    s_Swerve.drive(new Translation2d(0, SwerveDrive.dpadSpeed), 0, true, true);
    if (stopCheck == true) {
      convertedStopCheck = false;
    } else {
      convertedStopCheck = true;
    }
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
    return convertedStopCheck;
  }
}
