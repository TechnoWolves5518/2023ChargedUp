// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class extendArm extends TrapezoidProfileSubsystem {
  /** Creates a new armExtender. */
  
  public static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);

  public static void setExtendMotor(TalonSRXControlMode Position, double position){
    armExtender.set(Position, position);
  }
        
  
  /** Create a new ArmSubsystem. */
  public extendArm() {
    super(new TrapezoidProfile.Constraints(SpecialFunctions.extendMaxVelocity, SpecialFunctions.extendMaxAcceleration));

    ArmFeedforward extendfeedforward = new ArmFeedforward(SpecialFunctions.kS, SpecialFunctions.kG, SpecialFunctions.kV, SpecialFunctions.kA);


  }
        
    @Override
    public void useState(TrapezoidProfile.State setpoint) {

      State startPosition = new State(0, 0)

      TrapezoidProfile TrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          startPosition
          );

          }
  }