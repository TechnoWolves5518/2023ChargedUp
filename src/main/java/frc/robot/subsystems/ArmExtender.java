// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class ArmExtender extends SubsystemBase {
  private static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  private static Encoder extensionEncoder = new Encoder(SpecialFunctions.extendA, SpecialFunctions.extendB);
  private final static TrapezoidProfile.Constraints extendConstraints = new TrapezoidProfile.Constraints(
    SpecialFunctions.extendMaxVelocity, 
    SpecialFunctions.spinMaxAcceleration);
    
    public final ProfiledPIDController extendController =  new ProfiledPIDController(SpecialFunctions.extendKP, 
    SpecialFunctions.extendKI, 
    SpecialFunctions.extendKD, 
    extendConstraints);


  public ArmExtender() {
    extensionEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);
    armExtender.overrideLimitSwitchesEnable(true);
  }


  public void setMotors(TalonSRXControlMode mode,double speed) {
    //speed = extendController.calculate(extensionEncoder.getDistance());
    armExtender.set(mode, speed);
  }
}
