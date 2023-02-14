// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armExtender extends ProfiledPIDSubsystem {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  public static Encoder extendEncoder = new Encoder(SpecialFunctions.extendA, SpecialFunctions.extendB);


  /** Create a new Motor Subsystem */
  public static void setExtendMotor(TalonSRXControlMode mode, double position){
    armExtender.set(TalonSRXControlMode.Position, position);
  }

  public ArmFeedforward extendFeedForwards = new ArmFeedforward(SpecialFunctions.eSVolts, SpecialFunctions.eGVolts,
                                                                SpecialFunctions.eVVoltSecondPerRad, SpecialFunctions.eAVoltSecondSquaredPerRad);
        
  
  /** Create a new Arm Subsystem. */
  public armExtender() { 
    super(new ProfiledPIDController(0,
                                    0,
                                    0,
    new TrapezoidProfile.Constraints(SpecialFunctions.extendMaxVelocity, SpecialFunctions.extendMaxAcceleration)));

    extendEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);

  }
        
  @Override
  protected void useOutput(double output, State setpoint) {
    double finalfeedforward = extendFeedForwards.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    setExtendMotor(TalonSRXControlMode.Position, output + finalfeedforward);
  }


  @Override
  protected double getMeasurement() {
    return extendEncoder.getDistance() + SpecialFunctions.spinOffset;
  }
      

}