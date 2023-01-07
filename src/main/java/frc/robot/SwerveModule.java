package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    //define global variable names
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mTurnMotor;
    private TalonFX mDrivemotor;
    private CANCoder turnEncoder;
    private double lastAngle;
    //use PID values to get a new value
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotMap.driveKS, RobotMap.driveKV, RobotMap.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        //define module by module values
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        //turn encoder config
        turnEncoder = new CANCoder(moduleConstants.cancoderID);
        configTurnEncoder();
        
        //turn motor config
        mTurnMotor = new TalonFX(moduleConstants.turnMotorID);
        configTurnMotor();

        //drive motor config
        mDrivemotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //custom command as WPI lib optimizes a contiuous motor while the CTRE motors are not.
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / RobotMap.maxSpeed;
            mDrivemotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, RobotMap.wheelCircumference, RobotMap.driveGearRatio);
            mDrivemotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (RobotMap.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //prevent movement of turn module below 1% speed to prevent jittering
        mTurnMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, RobotMap.turnGearRatio));
        lastAngle = angle;
    } 
    //Create the custom functions for the above function to work
    private void resetToAbsolute(){
        double absolutePosotion = Conversions.degreesToFalcon(turnEncoder.getAbsolutePosition() - angleOffset, RobotMap.turnGearRatio);
        //System.out.println("absolute position: " + absolutePosotion);
        //System.out.println("Cancoder Degrees: " + turnEncoder.getAbsolutePosition());
        mTurnMotor.setSelectedSensorPosition(absolutePosotion);
    }

    private void configTurnEncoder(){
        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configTurnMotor(){
        mTurnMotor.configFactoryDefault();
        mTurnMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mTurnMotor.setInverted(RobotMap.turnMotorInvert);
        mTurnMotor.setNeutralMode(RobotMap.turnNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDrivemotor.configFactoryDefault();
        mDrivemotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDrivemotor.setInverted(RobotMap.driveMotorInvert);
        mDrivemotor.setNeutralMode(RobotMap.driveNeutralMode);
        mDrivemotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDrivemotor.getSelectedSensorVelocity(), RobotMap.wheelCircumference, RobotMap.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mTurnMotor.getSelectedSensorPosition(), RobotMap.turnGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
}