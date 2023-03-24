// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmExtender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSRX;

public class TestRetract extends CommandBase {
  TestSRX t_test;
  public TestRetract(TestSRX t_test) {
    this.t_test = t_test;
    addRequirements(t_test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t_test.setMotors(-0.6); //supposed to be 0.6
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t_test.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
