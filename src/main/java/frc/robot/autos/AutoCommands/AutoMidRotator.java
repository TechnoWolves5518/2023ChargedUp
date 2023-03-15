// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.armRotator.GoToStageOne;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.HandGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMidRotator extends ParallelDeadlineGroup {
  ArmSpinner a_Spinner;
  BrakeArm b_Arm;
  HandGripper h_Gripper;
  public AutoMidRotator(ArmSpinner a_Spinner, BrakeArm b_Arm, HandGripper h_Gripper) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new GoToStageOne(a_Spinner, b_Arm));
    addCommands(new autoClaw(h_Gripper));
  }
}
