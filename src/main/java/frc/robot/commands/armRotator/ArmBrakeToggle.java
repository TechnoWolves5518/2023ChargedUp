// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armRotator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BrakeArm;

public class ArmBrakeToggle extends CommandBase {
  BrakeArm b_arm;
  public ArmBrakeToggle(BrakeArm b_arm) {
    this.b_arm = b_arm;
    addRequirements(b_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    b_arm.BrakeOff();
    System.out.println("piston should be on");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    b_arm.BrakeOn();
    System.err.println("piston off");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
