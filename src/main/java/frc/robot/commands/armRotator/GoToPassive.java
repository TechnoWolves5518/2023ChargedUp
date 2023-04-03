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

public class GoToPassive extends CommandBase {
  BrakeArm b_Arm;
  ArmSpinner a_Spinner;
  ArmExtender a_Extender;
  int timer;
  boolean stopCheck;
  double armAngle;

  public GoToPassive(BrakeArm b_Arm, ArmSpinner a_Spinner, ArmExtender a_Extender) {
    this.b_Arm = b_Arm;
    this.a_Extender = a_Extender;
    this.a_Spinner = a_Spinner;
    addRequirements(b_Arm,a_Extender,a_Spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    stopCheck = false;
    b_Arm.BrakeOff();
    a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngle = a_Spinner.getAngle();
    timer++;
    if (armAngle == 0) {
      stopCheck = true;
    }
    if (timer < 40) {
      timer++;
      if (a_Extender.ReadRetractLimitSwitch() == true) {
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
      } else {
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
      }
    } else {
      if (armAngle > SpecialFunctions.pickupStage) {
        a_Spinner.setMotors(-SpecialFunctions.armSpeed);
        if (a_Extender.ReadRetractLimitSwitch() == true) {
          a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
        } else {
          a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
        }
    }else if (armAngle < SpecialFunctions.passiveStage -1) {
      a_Spinner.setMotors(-SpecialFunctions.armSpeed);
      if (a_Extender.ReadRetractLimitSwitch() == true) {
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
      } else {
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
      }
    } else if (armAngle > SpecialFunctions.passiveStage +1) {
      a_Spinner.setMotors(0.1);
      if (a_Extender.ReadRetractLimitSwitch() == true) {
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
      } else {  
        a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
      }
    } else {
      stopCheck = true;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a_Spinner.setMotors(0);
    a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
    b_Arm.BrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
