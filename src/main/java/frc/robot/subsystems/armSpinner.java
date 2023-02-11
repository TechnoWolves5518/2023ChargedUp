// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armSpinner extends TrapezoidProfileSubsystem { 

  public static  CANVenom armPwVenomLead = new CANVenom(SpecialFunctions.armOne);
  public static  CANVenom armPwmVenomTwo = new CANVenom(SpecialFunctions.armTwo);
  public static  CANVenom armPwmVenomThree = new CANVenom(SpecialFunctions.armThree);

  static double armSpeed;


    /*Sync Motors */
    

  public static void setSpinMotor(){
    armPwVenomLead.set(armSpeed);
    armPwmVenomTwo.follow(armPwVenomLead);
    armPwmVenomThree.follow(armPwVenomLead);
  }



  

  /** Create a new Spinner Subsystem. */
  public armSpinner(){
    super(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration));
    
    



    /* Feedforward system*/

    ArmFeedforward spinfeedforward = new ArmFeedforward(SpecialFunctions.kS, SpecialFunctions.kG, SpecialFunctions.kV, SpecialFunctions.kA);

  }


  @Override




  public void useState(final TrapezoidProfile.State setpoint) {

    State startPosition;

    TrapezoidProfile TrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, SpecialFunctions.spinMaxAcceleration),
          startPosition
          );

  }

}