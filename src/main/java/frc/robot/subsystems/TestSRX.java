// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class TestSRX extends SubsystemBase {
  TalonSRX testRedline;
  public TestSRX() {
    testRedline = new TalonSRX(SpecialFunctions.armExtender);
  }

  
  public void setMotors(double speed) {
    testRedline.set(TalonSRXControlMode.PercentOutput, speed);
  }
}
