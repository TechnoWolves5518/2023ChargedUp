// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class BrakeArm extends SubsystemBase {
  Solenoid armBrake;
  public BrakeArm() {
    armBrake = new Solenoid(21, PneumaticsModuleType.REVPH, SpecialFunctions.brakeSolenoid);
    armBrake.set(false);
  }

  public void BrakeOff() {
    armBrake.set(true);
  }

  public void BrakeOn() {
    armBrake.set(false);
  }
  public void BrakeToggle() {
    armBrake.toggle();
  }
}
