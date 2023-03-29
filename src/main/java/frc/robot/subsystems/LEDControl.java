// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDControl extends SubsystemBase {
  DigitalOutput ledOne;
  DigitalOutput ledTwo;
  DigitalOutput ledThree;

  public LEDControl() {
    ledOne = new DigitalOutput(Constants.ledDIOOne);
    ledTwo = new DigitalOutput(Constants.ledDIOTwo);
    ledThree = new DigitalOutput(Constants.ledDIOThree);
  }

  public void LEDOneOn() {
    ledOne.set(true);
  }

  public void LEDOneOff() {
    ledOne.set(false);
  }

  public void LEDTwoOn() {
    ledTwo.set(true);
  } 

  public void LEDTwoOff() {
    ledTwo.set(false);
  }

  public void LEDThreeOn() {
    ledThree.set(true);
  } 

  public void LEDThreeOff() {
    ledThree.set(false);
  }
}
