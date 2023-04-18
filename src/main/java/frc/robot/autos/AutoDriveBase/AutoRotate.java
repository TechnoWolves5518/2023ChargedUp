// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoDriveBase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoRotate extends CommandBase {
  Swerve s_Swerve;
  double timer;
  boolean stopCheck;
  public AutoRotate(Swerve s_Swerve) {
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
    s_Swerve.drive(new Translation2d(0,0), 1, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*s_Swerve.drive(new Translation2d(0,0), 
      0, 
      true, 
      true);*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
