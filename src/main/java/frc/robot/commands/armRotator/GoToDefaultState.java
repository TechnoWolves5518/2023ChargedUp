// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armRotator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;
import frc.robot.subsystems.HandGripper;

public class GoToDefaultState extends CommandBase {
  ArmSpinner a_Spinner;
  BrakeArm b_Arm;
  boolean stopCheck;
  boolean limitSwitchCheck;
  double previousArmAngle;
  ArmExtender a_ArmExtender;
  HandGripper h_Gripper;
  int timer;
  public GoToDefaultState(ArmSpinner a_Spinner, BrakeArm b_Arm, ArmExtender a_ArmExtender, HandGripper h_Gripper) {
    this.a_Spinner = a_Spinner;
    this.b_Arm = b_Arm;
    this.a_ArmExtender = a_ArmExtender;
    this.h_Gripper = h_Gripper;
    addRequirements(a_Spinner, b_Arm);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    previousArmAngle = a_Spinner.getAngle();
    timer = 0;
    h_Gripper.ForceOpen();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    previousArmAngle = a_Spinner.getAngle();
    if (timer < 40) {
      timer++;
      a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
    } else if (previousArmAngle > SpecialFunctions.stageOne) {
      a_Spinner.setMotors(SpecialFunctions.armSpeed);
      if (a_ArmExtender.ReadRetractLimitSwitch() == true ) {
        a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, 0);
      } else {
        a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -0.7);
      }
    } else {
    a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
    a_Spinner.setMotors(0.1);
    b_Arm.BrakeOff();
    if (a_ArmExtender.ReadRetractLimitSwitch() == true ) {
      a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, 0);
    } else {
      a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -0.7);
    }
    timer++;
    if (previousArmAngle -1 < SpecialFunctions.defaultStage) {
      stopCheck = true;
    } 
    if (timer == 200) {
      stopCheck = true;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  b_Arm.BrakeOn();
  a_Spinner.setMotors(0);
  a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
