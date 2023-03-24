// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandGripper;

public class AutoOpen extends CommandBase {
  HandGripper h_Gripper;
  boolean stopCheck;
  int timer;
  public AutoOpen(HandGripper h_Gripper) {
    this.h_Gripper = h_Gripper;
    addRequirements(h_Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    stopCheck = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer < 70) {
      timer++;
    } else {
    h_Gripper.ForceOpen();
    stopCheck = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
