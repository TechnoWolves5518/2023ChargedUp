// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;

public class ArmSpinner extends SubsystemBase {
  CANVenom venom1;
  CANVenom venom2;
  CANVenom venom3;
  //edu.wpi.first.wpilibj.Encoder spinEncoder;
  public ArmSpinner() {
    venom1 = new CANVenom(SpecialFunctions.armOne);
    venom1.enableLimitSwitches(true, true);
    venom2 = new CANVenom(SpecialFunctions.armTwo);
    venom2.enableLimitSwitches(true, true);
    venom3 = new CANVenom(SpecialFunctions.armThree);
    venom3.enableLimitSwitches(true, true);
    //spinEncoder = new edu.wpi.first.wpilibj.Encoder(SpecialFunctions.spinA, SpecialFunctions.spinB);
    //define encoder rotational value
    //spinEncoder.setDistancePerPulse(SpecialFunctions.spinRatio);
  }

  //define trapezoidal profile
  private final static TrapezoidProfile.Constraints spinConstraints = 
    new TrapezoidProfile.Constraints(SpecialFunctions.spinMaxVelocity, 
    SpecialFunctions.spinMaxAcceleration);

    public final static ProfiledPIDController spinController =  
    new ProfiledPIDController(SpecialFunctions.spinKP, 
                              SpecialFunctions.spinKI, 
                              SpecialFunctions.spinKD, 
                              spinConstraints);
 
  public void setMotors(double speed) {
    venom1.set(speed);
    venom2.follow(venom1);
    venom3.follow(venom1);
  }

  public void encoderCount() {
    //setMotors(spinController.calculate(spinEncoder.getDistance()));
  }


}
