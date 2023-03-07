// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class HandGripper extends SubsystemBase {
  DoubleSolenoid Gripper;
  public HandGripper() {
    Gripper = new DoubleSolenoid(21,PneumaticsModuleType.REVPH, SpecialFunctions.handClose, SpecialFunctions.handOpen);
    Gripper.set(Value.kReverse);
  }

  public void toggleState() {
    Gripper.toggle();
  }
}
