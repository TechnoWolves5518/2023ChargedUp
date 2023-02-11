// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class extendArm extends TrapezoidProfileSubsystem {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);

  public static void setMotor(TalonSRXControlMode Position, double position, double velocity){
    armExtender.set(Position, position);
  }
          
        
  /** Create a new ArmSubsystem. */
  public extendArm() {
    super(
      new TrapezoidProfile.Constraints( 
        SpecialFunctions.kMaxVelocityRadPerSecond, SpecialFunctions.kMaxAccelerationRadPerSecSquared),
      SpecialFunctions.extendOffset);
    }
        
    @Override
    public void useState(TrapezoidProfile.State setpoint) {

          }
  }