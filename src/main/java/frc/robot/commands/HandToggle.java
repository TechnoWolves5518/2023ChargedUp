// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandGripper;

public class HandToggle extends CommandBase {
  HandGripper h_Gripper;
  boolean stopCheck;
  public HandToggle(HandGripper h_Gripper) {
    this.h_Gripper = h_Gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    h_Gripper.toggleState();
    System.out.println("hand toggled");
    stopCheck = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stopCheck = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
