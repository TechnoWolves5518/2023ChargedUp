// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.armExtender;

public class retractIn extends CommandBase {
  /** Creates a new extendOut. */
  public retractIn() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.a_armExtender);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armExtender.extendController.setGoal(SpecialFunctions.fullRetract);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armExtender.setExtendMotor(TalonSRXControlMode.Position, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armExtender.extendController.atGoal() == true) {
    return true;
    } else { 
    return false;
    }
  }
}