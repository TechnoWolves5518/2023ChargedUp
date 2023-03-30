// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.AutoCommands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpecialFunctions;
import frc.robot.subsystems.ArmExtender;
import frc.robot.subsystems.ArmSpinner;
import frc.robot.subsystems.BrakeArm;

public class AutoGroundPickup extends CommandBase {
  ArmSpinner a_Spinner;
  BrakeArm b_Arm;
  ArmExtender a_Extender;
  boolean stopCheck;
  double previousArmAngle;
  public AutoGroundPickup(ArmSpinner a_Spinner, BrakeArm b_Arm, ArmExtender a_Extender) {
    this.a_Spinner = a_Spinner;
    this.b_Arm = b_Arm;
    this.a_Extender = a_Extender;
    addRequirements(a_Spinner, b_Arm, a_Extender);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    b_Arm.BrakeOff();
    stopCheck = false;
    previousArmAngle = a_Spinner.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (a_Extender.ReadRetractLimitSwitch() == true ) {
      a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
    } else {
      a_Extender.setMotors(TalonSRXControlMode.PercentOutput, -0.7);
    }
    previousArmAngle = a_Spinner.getAngle();
    if (previousArmAngle == 0) {
      stopCheck = true;
    }
    if (previousArmAngle < SpecialFunctions.passiveStage -1) {
      a_Spinner.setMotors(-SpecialFunctions.armSpeed);
    } else if (previousArmAngle > SpecialFunctions.passiveStage + 1) {
      a_Spinner.setMotors(SpecialFunctions.armSpeed);
    } else {
      stopCheck = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  b_Arm.BrakeOn();
  a_Spinner.setMotors(0);
  a_Extender.setMotors(TalonSRXControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
