// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends TrapezoidProfileSubsystem {

  public static CANVenom armPwVenomLead = new CANVenom(SpecialFunctions.armOne);
  public static CANVenom armPwmVenomTwo = new CANVenom(SpecialFunctions.armTwo);
  public static CANVenom armPwmVenomThree = new CANVenom(SpecialFunctions.armThree);  
  

  /** Create a new Spinner Subsystem. */
  public armSpinner() {

    /*Sync Motors */
    armPwmVenomTwo.follow(armPwVenomLead);
    armPwmVenomThree.follow(armPwVenomLead);

    /* Trapezoid Locations*/
    State fullyRotatedForward = new TrapezoidProfile.State(0, 0);
    State stopIntheMiddle = new TrapezoidProfile.State(0, 0);
    State fullyRotatedBackwards = new TrapezoidProfile.State(0, 0);

    /* Trapezoids to Move */
    final TrapezoidProfile spinMiddle_Front = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          stopIntheMiddle,
          fullyRotatedForward
          );

    final TrapezoidProfile spinMiddle_Back = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          stopIntheMiddle,
          fullyRotatedBackwards
          );

    final TrapezoidProfile spinBack_Middle = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          fullyRotatedForward,
          stopIntheMiddle
          );

    final TrapezoidProfile spinForward_Middle = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          fullyRotatedForward,
          stopIntheMiddle
          );
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
  }

}