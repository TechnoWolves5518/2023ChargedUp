// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve.SpecialFunctions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class rotateArmForward extends CommandBase {
  /** Creates a new moveArm. */
  XboxController specialSpinner = RobotContainer.special;
  

  public rotateArmForward() {
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
    boolean forwards = specialSpinner.getAButton();

    if (forwards == true){
      SpecialFunctions.armPwmVenom.set(SpecialFunctions.spinSpeed);

    } else {
      SpecialFunctions.armPwmVenom.set(0);
    }


    

    








  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; }
}