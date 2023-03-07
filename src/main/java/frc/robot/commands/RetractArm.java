// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;

public class RetractArm extends CommandBase {
  ArmExtender a_Extender;
  public RetractArm(ArmExtender a_Extender) {
    this.a_Extender = a_Extender;
    addRequirements(a_Extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    a_Extender.extendController.setGoal(SpecialFunctions.fullRetract);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_Extender.setMotors(TalonSRXControlMode.Position,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (a_Extender.extendController.atGoal() == true) {
      return true;
    } else {
    return false;
    }
  }
}
