// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.venomTest;

public class spinMotor extends CommandBase {
  double speed;
  venomTest venomTest;
  public spinMotor(venomTest venomTest) {
    this.venomTest = venomTest;
    addRequirements(venomTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0.05;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    venomTest.setMotors(speed);
    System.out.println("spinning");
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
/* */