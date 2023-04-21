// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmExtender;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;

public class ExtendArm extends CommandBase {
  private ArmExtender a_Extender;
  boolean stopCheck;
  double previousEncoderCount;
  int timer;
  int overrideTimer;
  Joystick override;
  boolean convertedStopCheck;
  public ExtendArm(ArmExtender a_Extender) {
    this.a_Extender = a_Extender;
    addRequirements(a_Extender);
    override = new Joystick(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    stopCheck = override.getRawButton(1);
    if (stopCheck == true) {
      convertedStopCheck = false;
    } else {
      convertedStopCheck = true;
    }
    overrideTimer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (overrideTimer < 2) {
      overrideTimer++;
    } else {
      stopCheck = override.getRawButton(1);
    previousEncoderCount = a_Extender.ReadEncoder();
    a_Extender.setMotors(TalonSRXControlMode.PercentOutput, SpecialFunctions.extendMaxVelocity);
    System.out.println("Arm Encoder Value: " + previousEncoderCount);
    if (timer < 100) {
      timer++;
      System.out.println(timer);
    } else {
      stopCheck = false;
    }
     
    if (a_Extender.ReadExtendLimitSwitch() == true) {
      stopCheck = false;
  } 
  if (stopCheck == true) {
    convertedStopCheck = false;
  } else {
    convertedStopCheck = true;
  }
}
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_Extender.ResetEncoderExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return convertedStopCheck;
  }
}
