// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends TrapezoidProfileSubsystem {
  public static CANVenom armPwmVenomOne = new CANVenom(SpecialFunctions.armOne);
  public static CANVenom armPwmVenomTwo = new CANVenom(SpecialFunctions.armTwo);
  public static CANVenom armPwmVenomThree = new CANVenom(SpecialFunctions.armThree);  


  private final ArmFeedforward b_feedforward =
      new ArmFeedforward(
        SpecialFunctions.bSVolts, SpecialFunctions.bGVolts,
        SpecialFunctions.bVVoltSecondPerRad, SpecialFunctions.bAVoltSecondSquaredPerRad);

  public static void setMotors(double speed){
    armPwmVenomOne.set(speed);
    armPwmVenomTwo.set(speed);
    armPwmVenomThree.set(speed);
  }

  /** Create a new ArmSubsystem. */
  public armSpinner() {
    super(
        new TrapezoidProfile.Constraints(
          SpecialFunctions.bMaxVelocityRadPerSecond, SpecialFunctions.bMaxAccelerationRadPerSecSquared),
          SpecialFunctions.spinOffset);

        armPwmVenomOne.setPID(SpecialFunctions.bP, 0, 0, 0, 0);
  }

  @Override
  public void useState(TrapezoidProfile.State velocState) {
    // Calculate the feedforward from the sepoint
    double spinfeedforward = b_feedforward.calculate(velocState.velocity, spinfeedforward);
    // Add the feedforward to the PID output to get the motor output
    armPwmVenomOne.setSetpoint(
      CANVenom.setPID.
      , velocState.position, spinfeedforward / 12.0);
  }

  public Command setArmGoalCommand(double bArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(SpecialFunctions.spinOffset), this);
  }
}