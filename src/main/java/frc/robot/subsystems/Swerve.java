// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Robot;
import frc.robot.RobotMap;

//insert IMU library here
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  //place libraries as variables to simplify calls
  public SwerveDriveOdometry swerveOdemetry;
  public SwerveModule[] mSwerveMods;
  public ADIS16470_IMU gyro;

  public Swerve() {
    gyro = Robot.gyro;
    gyro.calibrate();
    zeroGyro(); //reset the gyro when the bot starts to make sure field oriented works properly

    swerveOdemetry = new SwerveDriveOdometry(RobotMap.swerveKinematics, getYaw());

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, RobotMap.Mod0.constants),
      new SwerveModule(1, RobotMap.Mod1.constants),
      new SwerveModule(2, RobotMap.Mod2.constants),
      new SwerveModule(3, RobotMap.Mod3.constants)
    };
  }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
      SwerveModuleState[] swerveModuleStates =
        RobotMap.swerveKinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw()
          )
          : new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation)
          );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotMap.maxSpeed);

      for(SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      }
    }

    //used for autonomous driving
    public void setModuleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RobotMap.maxSpeed);
      for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
      }
    }

    public Pose2d getPose() {
      return swerveOdemetry.getPoseMeters();
    }

    public void resetOdemetry(Pose2d pose) {
      swerveOdemetry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(SwerveModule mod : mSwerveMods){
        states[mod.moduleNumber] = mod.getState();
      }
      return states;
    }

    public void zeroGyro(){
      gyro.reset();
    }

    public Rotation2d getYaw() {
      double ypr = -gyro.getAngle();
      return (RobotMap.inverGyro) ? Rotation2d.fromDegrees(360 - ypr) : Rotation2d.fromDegrees(ypr);
    }

  @Override
  public void periodic() {
    swerveOdemetry.update(getYaw(), getStates());  

      for(SwerveModule mod : mSwerveMods){
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);   
  }
}

  public void drive(int i, int rotation, boolean fieldRelative, boolean openLoop) {
  }
}

