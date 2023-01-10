// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

  /** Creates a new DriveSubsystem. */
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private static ADIS16470_IMU gyro = Robot.gyro;

  public DriveSubsystem() {
    frontLeftModule = new SwerveModule(SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftCANCoderID, false, false, false, SwerveConstants.frontLeftCANCoderOffset);
    frontRightModule = new SwerveModule(SwerveConstants.frontRightDriveID, SwerveConstants.frontRightTurnID, SwerveConstants.frontRightCANCoderID, true, false, false, SwerveConstants.frontRightCANCoderOffset);
    backLeftModule = new SwerveModule(SwerveConstants.backLeftDriveID, SwerveConstants.backLeftTurnID, SwerveConstants.backLeftCANCoderID, false, false, false, SwerveConstants.backLeftCANCoderOffset);
    backRightModule = new SwerveModule(SwerveConstants.backRightDriveID, SwerveConstants.backRightTurnID, SwerveConstants.backRightCANCoderID, true, false, false, SwerveConstants.backRightCANCoderOffset);
  }

  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Returns the rotation of the robot in radians
   * @return
   */
  public double getHeading() {
    return (gyro.getAngle() % 360) * Math.PI / 180;
  }

  public Rotation2d getRotation2d() {
    double yaw = -gyro.getAngle();
    return Rotation2d.fromDegrees(360 - yaw);
  }

  /**
   * Set's the speed and rotation of the swerve modules.
   * @param desiredStates An array that contains the SwerveModuleState for each
   * module. It is in the order frontLeft, frontRight, backLeft, backRight.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxVelocity);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets all of the swerve modules speed and angle to 0.
   */
  public void stopModules() {
    SwerveModuleState stoppedModuleState = new SwerveModuleState(0, new Rotation2d());
    frontLeftModule.setDesiredState(stoppedModuleState);
    frontRightModule.setDesiredState(stoppedModuleState);
    backLeftModule.setDesiredState(stoppedModuleState);
    backRightModule.setDesiredState(stoppedModuleState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
