// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class useGrip extends CommandBase {
  /** Creates a new closeGrip. */
  public useGrip() {

    addRequirements(RobotContainer.a_armGripper, RobotContainer.i_intakeSpinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armGripper.closeHand();
    intakeSpinner.pullIn();

  }


  @Override
  public void end(boolean interrupted) {
    armGripper.openHand();
    intakeSpinner.pushOut();
  }
  public boolean isFinished() {
    return false;
  }

}
