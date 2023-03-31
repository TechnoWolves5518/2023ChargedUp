// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;

public class AutoRetractArm extends CommandBase {
  private ArmExtender a_Extender;
  boolean stopCheck;
  double previousEncoderCount;
  int timer;
  public AutoRetractArm(ArmExtender a_Extender) {
    this.a_Extender = a_Extender;
    addRequirements(a_Extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousEncoderCount = a_Extender.ReadEncoder();
    a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -0.7);
    System.out.println("arm should be retracting");
    if (a_Extender.ReadExtendLimitSwitch() == true) {
      stopCheck = true;
  } 
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_Extender.ResetEncoderBase();
    a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
