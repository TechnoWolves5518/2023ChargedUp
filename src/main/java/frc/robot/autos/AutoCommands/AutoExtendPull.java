// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ArmExtender.ExtendArm;
import frc.robot.commands.Hand.PullIn;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.HandSpinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoExtendPull extends ParallelDeadlineGroup {
  /** Creates a new AutoExtend. */
  public AutoExtendPull(ArmExtender a_Extender, HandSpinner h_Spinner) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutoExtendArm(a_Extender));
    addCommands(new PullIn(h_Spinner));
  }
}
