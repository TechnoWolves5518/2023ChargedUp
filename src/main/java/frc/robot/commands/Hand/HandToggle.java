// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandGripper;

public class HandToggle extends CommandBase {
  HandGripper h_Gripper;
  boolean stopCheck;
  Joystick override;
  boolean convertedStopCheck;
  int timer;
  public HandToggle(HandGripper h_Gripper) {
    this.h_Gripper = h_Gripper;
    addRequirements(h_Gripper);
    override = new Joystick(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = override.getRawButton(1);
    timer = 0;
    if (stopCheck == true) {
      convertedStopCheck = false;
      h_Gripper.toggleState();
    } else {
      convertedStopCheck = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return convertedStopCheck;
  }
}
