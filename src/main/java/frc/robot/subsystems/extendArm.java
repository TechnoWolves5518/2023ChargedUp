// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class extendArm extends ProfiledPIDSubsystem {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  public static Encoder extendEncoder = new Encoder(SpecialFunctions.extendA, SpecialFunctions.extendB);


  /** Create a new Motor Subsystem */
  public static void setExtendMotor(TalonSRXControlMode mode, double speed){
    armExtender.set(TalonSRXControlMode.Position, speed);
  }

  
  ArmFeedforward extendFeedforward = new ArmFeedforward(SpecialFunctions.sSVolts, SpecialFunctions.sGVolts,
          SpecialFunctions.sVVoltSecondPerRad, SpecialFunctions.sAVoltSecondSquaredPerRad);
        
  
  /** Create a new Arm Subsystem. */
  public extendArm() {
    super(new ProfiledPIDController(0,
    0,
    0, 
    new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration)));

    extendEncoder.setDistancePerPulse(SpecialFunctions.extendRatio);
    
  }
        
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = extendFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    double finalSpeed = output + feedforward;

    setExtendMotor(TalonSRXControlMode.Position, finalSpeed);
   }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return extendEncoder.getDistance() + SpecialFunctions.extendOffset;
  }
      

}