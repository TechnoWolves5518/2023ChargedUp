// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotors extends SubsystemBase {
  CANVenom venom1;
  CANVenom venom2;
  CANVenom venom3;

  public TestMotors() {
    venom1 = new CANVenom(14);
    venom1.enableLimitSwitches(true, true);
    venom2 = new CANVenom(15);
    venom2.enableLimitSwitches(true, true);
    venom3 = new CANVenom(16);
    venom3.enableLimitSwitches(true, true);
  }


  public void setMotors(double speed) {
    venom1.set(speed);
    venom2.set(speed);
    venom3.set(speed);
    System.out.println();
  }
}
