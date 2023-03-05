// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class HandSpinner extends SubsystemBase {
  CANSparkMax rightHand;
  CANSparkMax leftHand;
  public HandSpinner() {
    rightHand = new CANSparkMax(SpecialFunctions.rightIntakeGrip, MotorType.kBrushless);
    leftHand = new CANSparkMax(SpecialFunctions.leftIntakeGrip, MotorType.kBrushless);
  }

  
  public void setMotors(double speed) {
    rightHand.set(-speed);
    leftHand.set(speed);
  }
}
