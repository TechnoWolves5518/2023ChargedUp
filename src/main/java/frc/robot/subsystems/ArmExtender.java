// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpecialFunctions;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class ArmExtender extends SubsystemBase {
  private static TalonSRX armExtender = new TalonSRX(SpecialFunctions.armExtender);
  private final static TrapezoidProfile.Constraints extendConstraints = new TrapezoidProfile.Constraints(
    SpecialFunctions.extendMaxVelocity, 
    SpecialFunctions.spinMaxAcceleration);
    
    public final ProfiledPIDController extendController =  new ProfiledPIDController(SpecialFunctions.extendKP, 
    SpecialFunctions.extendKI, 
    SpecialFunctions.extendKD, 
    extendConstraints);


  public ArmExtender() {
    armExtender.overrideLimitSwitchesEnable(true);
    armExtender.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    ResetEncoderBase();
  }
  


  public void setMotors(TalonSRXControlMode mode,double speed) {
    //speed = extendController.calculate(extensionEncoder.getDistance());
    armExtender.set(mode, speed);
  }
  public void ResetEncoderBase() {
    armExtender.setSelectedSensorPosition(0);
  }

  public void ResetEncoderExtension() {
    armExtender.setSelectedSensorPosition(11800);
  }
  public boolean ReadRetractLimitSwitch() {
    return armExtender.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean ReadExtendLimitSwitch() {
    return armExtender.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double ReadEncoder() {
    return armExtender.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Extension: ", armExtender.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Retract Limit Switch Status", ReadRetractLimitSwitch());
    SmartDashboard.putBoolean("Extend Limit Switch Status", ReadExtendLimitSwitch());
  }
}
