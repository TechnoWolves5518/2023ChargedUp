// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Compressor extends SubsystemBase {
  edu.wpi.first.wpilibj.Compressor compressor;
  public Compressor() {
    compressor = new edu.wpi.first.wpilibj.Compressor(1, PneumaticsModuleType.REVPH);
  }

  public void CompressorStart() {
    compressor.enableDigital();
  }

  public void CompressorStop() {
    compressor.disable();
  }
}
