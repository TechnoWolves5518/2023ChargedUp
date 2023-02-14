// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends SubsystemBase { 

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

  private final static TrapezoidProfile.Constraints spinConstraints = new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration);

  public final static ProfiledPIDController spinController =  new ProfiledPIDController(SpecialFunctions.spinKP, SpecialFunctions.spinKI, SpecialFunctions.spinKD, spinConstraints);


  /** Create a new Spinner Subsystem. */
  public armSpinner(){spinEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);}

  @Override
  public void periodic(){setSpinMotor(spinController.calculate(spinEncoder.getDistance()));}

}