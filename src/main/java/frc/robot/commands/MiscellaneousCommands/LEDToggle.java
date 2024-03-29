// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscellaneousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDControl;

public class LEDToggle extends CommandBase {
  LEDControl l_Control;
  int caseNumber;
  public LEDToggle(LEDControl l_Control) {
    this.l_Control = l_Control;
    caseNumber = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    l_Control.LEDTwoOff();
    l_Control.LEDThreeOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    l_Control.LEDTwoOn();
    l_Control.LEDThreeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
