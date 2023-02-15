// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.*;

public class rotateForwards extends CommandBase {
  /** Creates a new rotateBackwards. */
  public rotateForwards() {
    addRequirements(RobotContainer.a_armSpinner);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSpinner.spinController.setGoal(SpecialFunctions.fullyRotatedForward);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSpinner.setSpinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // end early if you reach goal
    
    if (armSpinner.spinController.atGoal() == true) {
    return true;
    } else { 
    return false;
    }
  }
}
