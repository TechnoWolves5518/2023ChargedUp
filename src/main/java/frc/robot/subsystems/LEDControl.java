// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDControl extends SubsystemBase {
  DigitalOutput ledOne;

  public LEDControl() {
    ledOne = new DigitalOutput(Constants.ledDIO);
  }

  public void LEDON() {
    ledOne.set(true);
  }

  public void LEDOFF() {
    ledOne.set(false);
  }
}
