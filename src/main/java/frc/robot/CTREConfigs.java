package frc.robot;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class CTREConfigs {
    //define the variables for the swerve motor templates
    public TalonFXConfiguration swerveTurnFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        //initiallize the variables
        swerveTurnFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        //Turn Motor configiration
        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            RobotMap.turnEnableCurrentLimit,
            RobotMap.turnContinuousCurrentLimit,
            RobotMap.turnPeakCurrentLimit,
            RobotMap.turnPeakCurrentDuration);
        swerveTurnFXConfig.slot0.kP = RobotMap.turnKP;
        swerveTurnFXConfig.slot0.kI = RobotMap.turnKI;
        swerveTurnFXConfig.slot0.kD = RobotMap.turnKD;
        swerveTurnFXConfig.slot0.kF = RobotMap.turnKF;
        swerveTurnFXConfig.supplyCurrLimit = turnSupplyLimit;
        swerveTurnFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        //Drive Motor Configuration
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            RobotMap.driveEnableCurrentLimit,
            RobotMap.driveContinuousCurrentLimit,
            RobotMap.drivePeakCurrentLimit,
            RobotMap.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = RobotMap.driveKP;
        swerveDriveFXConfig.slot0.kI = RobotMap.driveKI;
        swerveDriveFXConfig.slot0.kD = RobotMap.driveKD;
        swerveDriveFXConfig.slot0.kF = RobotMap.driveKF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = RobotMap.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = RobotMap.closedLoopRamp;

        //Cancoder Configuration
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = RobotMap.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
