// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.SwerveDrive.SpecialFunctions;

public class armExtender extends TrapezoidProfileSubsystem {
  /** Creates a new armExtender. */
  public static TalonSRX armExtender;

  public armExtender() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        0);

    armExtender = new TalonSRX(SpecialFunctions.armExtender);
  }

    public static void setMotors(TalonSRXControlMode Position, double position){
        armExtender.set(Position, position);
    }
  
  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}