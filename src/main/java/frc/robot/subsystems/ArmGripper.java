// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class ArmGripper extends SubsystemBase {
  DoubleSolenoid leftGripper;
  DoubleSolenoid rightGripper;
  public ArmGripper() {
    leftGripper = new DoubleSolenoid(PneumaticsModuleType.REVPH, SpecialFunctions.leftClose, SpecialFunctions.rightClose);
    rightGripper = new DoubleSolenoid(PneumaticsModuleType.REVPH, SpecialFunctions.rightClose, SpecialFunctions.rightOpen);
  }

  public void DefaultState() {
   leftGripper.set(Value.kReverse);
   rightGripper.set(Value.kReverse);
  }
/* 
  public void CloseHand() {
    leftGripper.set(Value.kForward);
    rightGripper.set(Value.kForward);
  }
  */
  public void GripperToggle() {
    leftGripper.toggle();
    rightGripper.toggle();
  }
}
