// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Hand.HandToggle;
import frc.robot.subsystems.HandGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClose extends InstantCommand {
  HandGripper h_Gripper;
  public AutoClose(HandGripper h_Gripper) {
    this.h_Gripper = h_Gripper;
    addRequirements(h_Gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    h_Gripper.ForceClose();
  }
}
