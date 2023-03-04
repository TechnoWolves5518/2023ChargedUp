// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class IntakeSpinner extends SubsystemBase {
  CANSparkMax rightGrabber;
  CANSparkMax leftGrabber;
  public IntakeSpinner() {
    rightGrabber = new CANSparkMax(SpecialFunctions.rightIntakeGrip, MotorType.kBrushless);
    leftGrabber = new CANSparkMax(SpecialFunctions.leftIntakeGrip, MotorType.kBrushless);
  }


  public void setMotors(double speed) {
    rightGrabber.set(speed);
    leftGrabber.follow(rightGrabber, true);
  }
}