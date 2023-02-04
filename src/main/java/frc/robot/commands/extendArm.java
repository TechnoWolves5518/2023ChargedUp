// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve.SpecialFunctions;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class extendArm extends CommandBase {
  /** Creates a new extendArm. */
  XboxController specialSpinner = RobotContainer.special;
  
  public extendArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    final armExtender e_Extender = new armExtender(); 
    addRequirements(e_Extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean extend = specialSpinner.getLeftBumper();

    if (extend == true){
      armExtender.extendSystem(SpecialFunctions.extendSpeed);

    } else {
      armExtender.extendSystem(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}