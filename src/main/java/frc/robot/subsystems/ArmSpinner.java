// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SpecialFunctions;

public class ArmSpinner extends SubsystemBase {
  CANVenom venom1;
  CANVenom venom2;
  CANVenom venom3;
  CANCoder spinCoder;
  private double previousArmAngle;
  //edu.wpi.first.wpilibj.Encoder spinEncoder;
  public ArmSpinner() {
    venom1 = new CANVenom(SpecialFunctions.armOne);
    venom1.setBrakeCoastMode(BrakeCoastMode.Brake);
    //venom1.enableLimitSwitches(true, true);
    venom2 = new CANVenom(SpecialFunctions.armTwo);
    venom2.setBrakeCoastMode(BrakeCoastMode.Brake);
    //venom2.enableLimitSwitches(true, true);
    venom3 = new CANVenom(SpecialFunctions.armThree);
    venom3.setBrakeCoastMode(BrakeCoastMode.Brake);
    //venom3.enableLimitSwitches(true, true);
    spinCoder = new CANCoder(SpecialFunctions.spinEncoder);
    spinCoder.configFactoryDefault();
    spinCoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
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

  public double getAngle() {
    return spinCoder.getAbsolutePosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle: ", spinCoder.getAbsolutePosition());
  }

}
