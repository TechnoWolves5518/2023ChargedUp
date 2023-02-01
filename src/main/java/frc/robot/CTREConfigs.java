package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrive.angleEnableCurrentLimit, 
            Constants.SwerveDrive.angleContinuousCurrentLimit, 
            Constants.SwerveDrive.anglePeakCurrentLimit, 
            Constants.SwerveDrive.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveDrive.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveDrive.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveDrive.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveDrive.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrive.driveEnableCurrentLimit, 
            Constants.SwerveDrive.driveContinuousCurrentLimit, 
            Constants.SwerveDrive.drivePeakCurrentLimit, 
            Constants.SwerveDrive.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveDrive.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveDrive.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveDrive.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveDrive.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveDrive.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveDrive.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveDrive.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}