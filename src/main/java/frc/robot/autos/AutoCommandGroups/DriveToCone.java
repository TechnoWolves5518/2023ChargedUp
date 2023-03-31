// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.AutoCommands.AutoGroundPickup;
import frc.robot.commands.armRotator.GoToPassiveStage;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToCone extends ParallelCommandGroup {

  public DriveToCone(Swerve drivebase, ArmSpinner a_Spinner, BrakeArm b_Arm, ArmExtender a_Extender) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DelayedPassive(drivebase), new AutoGroundPickup(a_Spinner, b_Arm, a_Extender));
  }
}
