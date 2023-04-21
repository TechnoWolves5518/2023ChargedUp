// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armRotator;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;

public class GoToPassiveStage extends CommandBase {
  ArmSpinner a_Spinner;
  BrakeArm b_Arm;
  boolean stopCheck;
  double previousArmAngle;
  ArmExtender a_ArmExtender;
  int timer;
  Joystick override;
  boolean convertedStopCheck;
  int overrideTimer;

  public GoToPassiveStage(ArmSpinner a_Spinner, BrakeArm b_Arm, ArmExtender a_ArmExtender) {
    this.a_Spinner = a_Spinner;
    this.b_Arm = b_Arm;
    this.a_ArmExtender = a_ArmExtender;
    addRequirements(a_Spinner, b_Arm);
    override = new Joystick(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = override.getRawButton(1);
    if (stopCheck == true) {
      convertedStopCheck = false;
    } else {
      convertedStopCheck = true;
    }
    overrideTimer = 0;
    previousArmAngle = a_Spinner.getAngle();
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (overrideTimer < 2) {
      overrideTimer++;
    } else {
      stopCheck = override.getRawButton(1);
    previousArmAngle = a_Spinner.getAngle();
    a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
    if (timer < 20) {
      timer++;
    } else if (previousArmAngle > SpecialFunctions.stageTwo) {
      a_Spinner.setMotors(SpecialFunctions.armSpeed);
    } else {
    a_ArmExtender.setMotors(TalonSRXControlMode.PercentOutput, -.7);
    a_Spinner.setMotors(0.1);
    b_Arm.BrakeOff();
    timer++;
    if (previousArmAngle -1 < SpecialFunctions.passiveStage - SpecialFunctions.armDrift && previousArmAngle + 1 > SpecialFunctions.passiveStage - SpecialFunctions.armDrift) {
      stopCheck = false;
    } 
    if (timer == 145) {
      stopCheck = false;
    }
    if (stopCheck == true) {
      convertedStopCheck = false;
    } else {
      convertedStopCheck = true;
    }
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
    return convertedStopCheck;
  }
}
