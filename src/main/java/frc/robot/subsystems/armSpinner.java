// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends TrapezoidProfileSubsystem { 

  public static  CANVenom armPwVenomLead = new CANVenom(SpecialFunctions.armOne);
  public static  CANVenom armPwmVenomTwo = new CANVenom(SpecialFunctions.armTwo);
  public static  CANVenom armPwmVenomThree = new CANVenom(SpecialFunctions.armThree);
  public static Encoder spinEncoder = new Encoder(SpecialFunctions.spinA, SpecialFunctions.spinB);
  
  /*Sync Motors */  
  public static void setSpinMotor(double speed){
    armPwVenomLead.set(speed);
    armPwmVenomTwo.follow(armPwVenomLead);
    armPwmVenomThree.follow(armPwVenomLead);
  }


  /** Create a new Spinner Subsystem. */
  public armSpinner(){
    super(new ProfiledPIDController(
      0,
      0,
      0, 
      new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration));

    double spinDistance = 0.0;
    
    spinEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);

    State spinStartPosition = new State(spinDistance, 0);

    TrapezoidProfile spinProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          spinStartPosition
          );

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return spinEncoder.getDistance() + SpecialFunctions.spinOffset;
  }

}