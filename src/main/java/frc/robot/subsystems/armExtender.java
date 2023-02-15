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

public class armExtender extends SubsystemBase {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  public static Encoder extendEncoder = new Encoder(SpecialFunctions.extendA, SpecialFunctions.extendB);


  /** Move Motor */
  public static void setExtendMotor(TalonSRXControlMode mode, double speed){
    armExtender.set(TalonSRXControlMode.Position, speed);
  }

  private final static TrapezoidProfile.Constraints extendConstraints = new TrapezoidProfile.Constraints(
    SpecialFunctions.extendMaxVelocity, 
    SpecialFunctions.spinMaxAcceleration);

  public final static ProfiledPIDController extendController =  new ProfiledPIDController(SpecialFunctions.extendKP, 
                                                                                          SpecialFunctions.extendKI, 
                                                                                          SpecialFunctions.extendKD, 
                                                                                          extendConstraints);
 
  /** Create a new armExtender Subsystem. */
  public armExtender() {
    extendEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);
  }

  @Override
  public void periodic(){
    setExtendMotor(TalonSRXControlMode.Position, extendController.calculate(extendEncoder.getDistance()));
  }
      
}