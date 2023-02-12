// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class extendArm extends ProfiledPIDSubsystem {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  public static Encoder extendEncoder = new Encoder(SpecialFunctions.extendA, SpecialFunctions.extendB);


  /** Create a new Motor Subsystem */
  public static void setExtendMotor(TalonSRXControlMode mode, double position){
    armExtender.set(TalonSRXControlMode.Position, position);
  }
        
  
  /** Create a new Arm Subsystem. */
  public extendArm() {
    super(new ProfiledPIDController(0,
    0,
    0, 
    new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration));new TrapezoidProfile.Constraints(SpecialFunctions.extendMaxVelocity, SpecialFunctions.extendMaxAcceleration));

    extendEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);

    double extendDistance = 0.0;

    State extendStartPosition = new State(extendDistance, 0);

      TrapezoidProfile extendProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          extendStartPosition
          );
  }
        
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return extendEncoder.getDistance() + SpecialFunctions.extendOffset;
  }
      

}