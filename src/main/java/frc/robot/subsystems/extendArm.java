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

  public static void setMotor(TalonSRXControlMode Position, double position, double velocity){
    armExtender.set(Position, position);
  }

  private final Encoder m_encoder =
      new Encoder(SpecialFunctions.kEncoderPorts[0], SpecialFunctions.kEncoderPorts[1]);
  
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        SpecialFunctions.kSVolts, SpecialFunctions.kGVolts,
        SpecialFunctions.kVVoltSecondPerRad, SpecialFunctions.kAVoltSecondSquaredPerRad);


  public extendArm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            SpecialFunctions.kP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(SpecialFunctions.extendMaxVelocity, SpecialFunctions.extendMaxAcceleration)));
  
    // Start arm at rest in neutral position
        setGoal(SpecialFunctions.extendOffset);

  }

}
  

@Override
public void useOutput(double output, TrapezoidProfile.State setpoint) {
  
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    extendArm.setMotor(output + feedforward);
}

@Override
public double getMeasurement() {
  return m_encoder.getDistance() + SpecialFunctions.extendOffset;
}