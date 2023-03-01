// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestMotors;

public class SpinVenoms extends CommandBase {
  double speed;
  TestMotors motors;
  public SpinVenoms(TestMotors m_Motors) {
    this.motors = m_Motors;
    addRequirements(m_Motors);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    speed = 0.05;
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    motors.setMotors(speed);
    System.out.println("spinning");
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
