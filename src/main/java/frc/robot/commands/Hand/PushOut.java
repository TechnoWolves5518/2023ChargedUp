// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.HandSpinner;

public class PushOut extends CommandBase {
  HandSpinner h_Spinner;
  Joystick override;
  boolean stopCheck;
  boolean convertedStopCheck;
  int timer;
  public PushOut(HandSpinner h_Spinner) {
    this.h_Spinner = h_Spinner;
    addRequirements(h_Spinner);
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
    h_Spinner.setMotors(-SpecialFunctions.handSpeed);
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
    h_Spinner.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return convertedStopCheck;
  }
}

