// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmExtender.ExtendArm;
import frc.robot.commands.armRotator.GoToDefaultState;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.HandGripper;
import frc.robot.subsystems.HandSpinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHighScore extends SequentialCommandGroup {
  ArmSpinner a_Spinner;
  BrakeArm b_Arm;
  HandGripper h_Gripper;
  HandSpinner h_Spinner;
  ArmExtender a_Extender;
  public AutoHighScore(ArmSpinner a_Spinner, BrakeArm b_Arm, HandGripper h_Gripper, HandSpinner h_Spinner, ArmExtender a_Extender) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoMidRotator(a_Spinner, b_Arm, h_Gripper, h_Spinner), 
    new AutoExtend(a_Extender, h_Spinner), 
    new AutoOpen(h_Gripper),
    new DelayAction());
  }
}
