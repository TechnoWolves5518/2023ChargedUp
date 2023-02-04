// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;
import frc.robot.subsystems.*;


public class rotateArmBackward extends CommandBase {
  /** Creates a new moveArm. */
  public rotateArmBackward() {
    // Use addRequirements() here to declare subsystem dependencies

    final armSpinner s_Spin = new armSpinner(); 
    addRequirements(s_Spin);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {      
      armSpinner.setMotors(-SpecialFunctions.spinSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSpinner.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; }
}